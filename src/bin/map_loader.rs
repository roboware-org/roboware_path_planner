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

use safe_drive::{
    context::Context, 
    error::DynError, 
    logger::Logger,
    pr_info, msg::common_interfaces::std_msgs,
};
use visualization_msgs::msg::MarkerSeq;
use safe_drive::msg::RosStringSeq;

use planning_util::{graph_map, vector};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

fn main() -> Result<(), DynError> {
    // ---- safe drive ----
    let ctx = Context::new()?;
    let node = ctx.create_node("map_loader", None, Default::default())?;

    // ---- ros2 topic ----
    let publisher_map_debug = node.create_publisher::<visualization_msgs::msg::MarkerArray>("map_graph", None)?;

    // ---- ros2 param ----

    
    // ---- Create a logger ----
    let logger = Logger::new("lock_arm_node");
    pr_info!(logger, "start node: {:?}", node.get_name());


    loop {
        let map = graph_map::map_loader("/home/taiga/planning_util/example.json");

        let mut array_size = 0;

        for graph in map.point_graph.iter(){
            for _ in graph.iter(){
                array_size += 1;
            }
        }
        
        let mut data = MarkerSeq::new(array_size).unwrap();
        let mut i = 0;

        for (index, graph) in map.point_graph.iter().enumerate() {
            for point in graph.iter() {
                let mut marker = visualization_msgs::msg::Marker::new().unwrap();
                marker.header.frame_id.assign("map");
                let now = SystemTime::now();
                let unixtime = now.duration_since(UNIX_EPOCH).expect("back to the future");
                marker.header.stamp.sec = unixtime.as_secs() as i32;
                marker.header.stamp.nanosec = unixtime.as_nanos() as u32;

                marker.type_ = 0; // arrow
                marker.action = 0;
                marker.id = i as i32;

                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                marker.color.a = 1.0;
                marker.color.g = 1.0;

                let point_base = vector::Vector3 { x: map.points[index].x , y: map.points[index].y , z: 0.0};
                let point_target = vector::Vector3 { x: point.x , y: point.y , z: 0.0};
                let quo: vector::Quaternion = vector::Quaternion::from_two_vectors(&point_base, &point_target);
                pr_info!(logger, "{:?},{:?},{:?}", quo, point_base, point_target);

                marker.pose.orientation.w = quo.w as f64;
                marker.pose.orientation.x = quo.x as f64;
                marker.pose.orientation.y = quo.y as f64;
                marker.pose.orientation.z = quo.z as f64;

                marker.pose.position.x = map.points[index].x as f64;
                marker.pose.position.y = map.points[index].y as f64;

                marker.scale.x = (graph_map::distance(&map.points[index], &point) as f64) / 1000.;

                data.as_slice_mut()[i] = marker;
                i += 1;
            }
        }

        let mut msg = visualization_msgs::msg::MarkerArray::new().unwrap();
        msg.markers = data;
        publisher_map_debug.send(&msg)?;
        std::thread::sleep(Duration::from_secs(1));
    }
}