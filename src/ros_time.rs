use std::time::{SystemTime, UNIX_EPOCH};

pub fn get_time() -> (i32 , u32){
    let now = SystemTime::now();
    let unixtime = now.duration_since(UNIX_EPOCH).expect("back to the future");

    return (unixtime.as_secs() as i32 , unixtime.as_nanos() as u32);
}