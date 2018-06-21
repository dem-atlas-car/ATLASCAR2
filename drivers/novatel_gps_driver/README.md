NovAtel GPS Driver 
==================

Adaptado por Pedro Bouça Nova

Nova menssagem; BESTGNSSPOS
Posição global proveniente do GPS.



Parâmetro; publish_default_messages_
Publicação dos tópicos /gps e /fix.



    <!-- Node novatel para comandos relativos a posição-->
    <node name="novatel_position" pkg="nodelet" type="nodelet" args="standalone novatel_gps_driver/novatel_gps_nodelet"><rosparam>
      connection_type: serial
      device: /dev/ttyUSB1
      publish_novatel_positions: true
      publish_novatelgnss_positions: true
      publish_imu_messages: false
      publish_nmea_messages: true
      publish_default_messages: true
      publish_diagnostics: true
      publish_novatel_velocity: true
      frame_id: /gps
  </rosparam>
  </node>
    <!-- Node novatel para comandos da unidade inercial -->
    <node name="novatel_imu" pkg="nodelet" type="nodelet" args="standalone novatel_gps_driver/novatel_gps_nodelet"><rosparam>
      connection_type: serial
      device: /dev/ttyUSB1
      publish_novatel_positions: false
      publish_novatelgnss_positions: false
      publish_imu_messages: true
      publish_nmea_messages: false
      publish_default_messages: false
      publish_diagnostics: true
      frame_id: /gps
  </rosparam></node>

