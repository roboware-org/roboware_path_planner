// Copyright 2023 Hakoroboken
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

use planning_util::dijkstra::Point;
use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info
};
use visualization_msgs::msg::MarkerSeq;

use planning_util::dijkstra;
use roboware_path_planner::{ros_time, visual};

use std::time::Duration;
use nalgebra::UnitQuaternion;


fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("map_loader", None, Default::default())?;

    // ---- ros2 topic ----
    let publisher_map_graph_debug = node.create_publisher::<visualization_msgs::msg::MarkerArray>("debug/map_graph", None)?;
    let publisher_point_debug = node.create_publisher::<visualization_msgs::msg::MarkerArray>("debug/point", None)?;

    // ---- ros2 param ----

    
    // ---- Create a logger ----
    let logger = Logger::new("lock_arm_node");
    pr_info!(logger, "start node: {:?}", node.get_name());

    loop {
        let map = dijkstra::map_loader("/Users/taigatakano/Desktop/planning_util/example.json");
        
        let mut graph_marker_data = MarkerSeq::new(map.graph_size()).unwrap();
        let mut i = 0;

        for (index, graph) in map.point_graph.iter().enumerate() {
            for point in graph.iter() {
                let mut marker = visualization_msgs::msg::Marker::new().unwrap();
                marker.header.frame_id.assign("map");
                (marker.header.stamp.sec, marker.header.stamp.nanosec) = ros_time::get_time();

                marker.type_ = 0;
                marker.action = 0;
                marker.id = i as i32;

                marker.scale.y = 0.05;
                marker.scale.z = 0.05;

                (marker.color.r, marker.color.g, marker.color.b, marker.color.a) = visual::get_green();

                let point_vec = &Point{ x: point.x - map.points[index].x, y: point.y - map.points[index].y};

                let yaw = (point_vec.y / point_vec.length()).asin();

                let quo = UnitQuaternion::from_euler_angles(0., 0., yaw.into());
                pr_info!(logger, "a:{:?},{:?}", quo, point_vec);
                pr_info!(logger, "b:{:?}", yaw.to_degrees());

                marker.pose.orientation.w = quo.w as f64;
                marker.pose.orientation.x = quo.i as f64;
                marker.pose.orientation.y = quo.j as f64;
                marker.pose.orientation.z = quo.k as f64;

                marker.pose.position.x = map.points[index].x.clone() as f64;
                marker.pose.position.y = map.points[index].y.clone() as f64;

                marker.scale.x = (dijkstra::distance(&map.points[index], &point) as f64) / 1000.;

                graph_marker_data.as_slice_mut()[i] = marker;
                i += 1;
            }
        }

        i = 0;
        let mut point_marker_data = MarkerSeq::new(map.graph_size()).unwrap();
        for (index, point) in map.points.iter().enumerate(){
            let mut marker = visualization_msgs::msg::Marker::new().unwrap();
            marker.header.frame_id.assign("map");
            (marker.header.stamp.sec, marker.header.stamp.nanosec) = ros_time::get_time();

            marker.type_ = 9;
            marker.action = 0;
            marker.id = i as i32;
            i += 1;

            (marker.color.r, marker.color.g, marker.color.b, marker.color.a) = visual::get_white();
            marker.pose.position.x = point.x.clone() as f64;
            marker.pose.position.y = point.y.clone() as f64;
            marker.scale.z = 0.2;
            let txt = point.x.to_string() + "," + point.y.to_string().as_str();
            marker.text.assign(txt.as_str());
            point_marker_data.as_slice_mut()[index] = marker;
        }

        let mut msg = visualization_msgs::msg::MarkerArray::new().unwrap();
        msg.markers = graph_marker_data;
        publisher_map_graph_debug.send(&msg)?;

        let mut msg2 = visualization_msgs::msg::MarkerArray::new().unwrap();
        msg2.markers = point_marker_data;
        publisher_point_debug.send(&msg2)?;
        std::thread::sleep(Duration::from_secs(1));
    }
}