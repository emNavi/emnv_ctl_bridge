#!/bin/bash
takeoff(){
  drone_id=$1
  rostopic pub /emnavi_cmd/takeoff std_msgs/String "data: '$drone_id'" -1
}
land(){
  drone_id=$1
  rostopic pub /emnavi_cmd/land std_msgs/String "data: '$drone_id'" -1
}
update_ctrl_params(){
  rostopic pub /emnavi_cmd/update_ctrl_params std_msgs/Empty "{}" -1
}