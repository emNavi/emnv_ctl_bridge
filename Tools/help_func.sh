takeoff(){
  drone_name=$1
  rostopic pub /emnavi_cmd/takeoff std_msgs/String "data: '$drone_name'"
}
land(){
  drone_name=$1
  rostopic pub /emnavi_cmd/land std_msgs/String "data: '$drone_name'"
}