takeoff(){
  drone_id=$1
  rostopic pub /emnavi_cmd/takeoff std_msgs/String "data: '$drone_id'"
}
land(){
  drone_id=$1
  rostopic pub /emnavi_cmd/land std_msgs/String "data: '$drone_id'"
}