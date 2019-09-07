1、roscore
2、打开rosserial python_rosserial node.py /dev/ttyUSB0
3、打开talker，单片机即可接收消息。同时可通过订阅chatter_vector读取talker的内容
4、打开chatter，订阅单片机发过来的内容，rostopic echo /chatter