^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package qualisys_bridge
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* actually use the ros parameters
* Contributors: Thies Lennart Alff

1.0.4 (2024-09-10)
------------------
* refactored reading the qtm config
* Contributors: Thies Lennart Alff

1.0.3 (2024-09-10)
------------------
* added missing process noise entries
* removed now erroneous namespace from launch file
* updated formmating
* major rework to handle multiple bodies in a single bridge
* revert commits related with a single body per node approach.
  QTM seems to not support multiple udp clients properly (not thoroughly
  verified though). Hence multiple/all bodies have to be handled inside a
  single node
* check body name only once in the beginning. bumped requested protocol version
* try several udp ports before giving up
* Contributors: Thies Lennart Alff

1.0.2 (2024-08-15)
------------------
* fixed regression caused by inversed parameter logic
* Contributors: Thies Lennart Alff

1.0.1 (2024-08-15)
------------------
* removed unused param. also renamed another to make intent more clear
* Contributors: Thies Lennart Alff

1.0.0 (2024-08-12)
------------------
* added missing include
* Contributors: Thies Lennart Alff

0.0.1 (2024-08-12)
------------------
* streamlined the qualisys bridge. removed px4 bridge
* added project settings and format configs
* set meaningful default node name
* handle latch and ignore timoeut settings
* added param to ignore mocap timeouts
* declare parameters without default
* added latch parameter
* fixed missing loading of config file
* added params/launch setup and changed velocity frames
* initial commit
* Contributors: Thies Lennart Alff
