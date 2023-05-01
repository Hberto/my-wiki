# Description
Ein eigener Wiki, um eine eigene Wissensbasis aufzustellen. Dies soll das wiederholte Googlen vermeiden

## Learning Path
- submodul bzw. fork in eigenes Repo reinpacken
- rplidar unter ros1 zum laufen bringen
- rplidar unter ros2 zum laufen bringen
- ros2 c++ wiederholen
- klassendiagramm mit interfaces implementieren

# ROS2 Overview
1. DDS (Data Distribution Service)
    - Kommunikationspipeline
    - 1.Möglichkeit: Publish-Subscribe
    - 2.Möglichkeit: Services
    - 3.Möglichkeit: Actions
        - Node legt ein Ziel fest, kriegt Feedback bis ein Ergebnis vorliegt
2. Ros Nodes 
3. Rosparam
4. Rospackages mit Funktionalitäten

## Fehler
- Bagfile = speichert Daten in eine Datenbank

## Unterschiede zwischen ROS1 und ROS2
1. DDS wird benutzt
    - kein ROS Master
    - Jede Node kann ohne Master und ohne Registrierung beim Parameter Server laufen
2. Security
3. Launch files von XML auf Python basiert
3. Bag File -> Speicherung in SQLite
4. ROS2 mit ROS1 Bridge Tool

# Workspaces
## Aufbau
ros_workspace_name
- src
    - package_1_name..
        - src (C++)
        - Package.xml (auto)
        - CMakeLists.txt (auto)


        - Scrips(Python)
        - launch
        - msg etc
    - package_2_name

- build/install/log (auto)

## Colcon (Command-line tool, was catkin ersetzt)
- Ersetzt das alte build system catkin
### Installation
```shell
sudo apt install python3-colcon-common-extensions
```
### Setup colcon_cd
- Beschreibung: The command colcon_cd allows you to quickly change the current working directory of your shell to the directory of a package. As an example colcon_cd some_ros_package would quickly bring you to the directory ~/ros2_ws/src/some_ros_package
- In deiner ROS2 Package
```shell
echo "source /usr/share/colcon_cd/function/colcon_cd.sh" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

### Setup colcon tab completion
```shell
echo "source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash" >> ~/.bashrc
echo "export _colcon_cd_root=/opt/ros/humble/" >> ~/.bashrc
```

### Colcon Build
- Workspace Ordner
```shell
colcon build
```


## Eigenen Workspace erstellen
### C++
```shell
ros2 pkg create dein_name --build-type ament_cmake
```

## Package List zeigen
```shell
ros2 pkg list
```

## Workspace bauen
- Workspace Ordner
```shell
colcon build
```

## Eigenen Package sourcen
- Im Install Ordner gibt es eine setup.bash die für unsere Umgebung sichtbar sein sollte
```shell
source install/setup.bash
```

# ROS Publisher

- Template, um einen ROS Publisher zu erstellen
## RCL Node Klasse
```cpp
#include "rclcpp/rclcpp.hpp"
class YourNodeClassName : public rclcpp::Node
    {
        public:
            YourNodeClassName() : Node("name_of_node"){...}
        private:
            ...
    };
int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<YourNodeClassName>());
        rclcpp::shutdown();
        return 0;
    }
```
## Publisher erstellen und diese benutzen
```cpp
//#include "std_msgs/msg/string.hpp”

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
publisher_ =
    this->create_publisher<std_msgs::msg::String>("topic_name", 10);

auto message = std_msgs::msg::String();
message.data = "My String Message Data"
publisher_->publish(message);
```

# Ros Nodes Compile

## CMAKELists.txt
### Dependencies hinzufügen
```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
```
### Executables konfiguieren
```cmake
add_executable(exe1_name src/your_ros_file.cpp)
ament_target_dependencies(exe1_name rclcpp std_msgs)

add_executable(exe2_name src/your_other_ros_file.cpp)
ament_target_dependencies(exe2_name rclcpp std_msgs)
```
### Targets installieren
```cmake
install(TARGETS
exe1_name
exe2_name
DESTINATION lib/${PROJECT_NAME}
)
```

## Package XML File
### Dependencies hinzufügen
```xml
<depend>rclcpp</depend>
<depend>std_msgs</depend>
```
## Im Terminal kompellieren
```shell
$ cd path/to/ros_workspace
$ colcon build
$ source install/setup.bash
$ ros2 run your_pkg_name exe1_name
```
## Ros Node nach Kompellierung laufen lassen
```shell
$ ros2 run your_pkg_name exe1_name
```

## Ros Topics
```shell
# Ros Topics anzeigen
$ ros2 topic list 
# Auf Data von einer Ros Topic hören
$ ros2 topic echo /hello_world
```

## Ros Nodes anzeigen
```shell
$ ros2 node list 
```

# Compile and Debug in VSCode
- mithilfe GDB Server
- Debuggen in VSCode muss verbunden werden 

## Im Terminal mit GDBServer
```shell
# Flags setzen fürs Debuggen
$ colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
$ ros2 run --prefix 'gdbserver localhost:3000' udemy_ros2_pkg publisher
```
## Launch File
- Angabe des Debug Servers
- Angabe wo der executable ist
- Verbindet sich mit den RUN-Befehlen

## Simulink
- Problem: Bei jedem Colcon Build werden Dateien wie executable, xml kopiert in den install ordner
- Wenn wir etwas ändern soll nicht jedes Mal neu gebuilded werden
- Lösung:
```shell
# Flags setzen fürs Debuggen
$ colcon build --symlink-install
# Die links lesen
$ readlink ...
```

# Ros Subscriber

- Template, um einen ROS Subscriber zu erstellen

## Subscriber erstellen
```cpp
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp”

rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

subscription_ = this->create_subscription<std_msgs::msg::String>(
                "topic_name", 10,
                std::bind(&YourNodeClassName::sub_callback,
                this, std::placeholders::_1)
```

## Subscriber Callback Funktion

```cpp
void sub_callback(const std_msgs::msg::String & msg) const
{
std::cout << msg.data << std::endl;
}
```

# Ros Logs
## Log Files
- unter ~/ros/log zu finden

## Logs Einsatz
```cpp
// ROS LOGS
			RCLCPP_INFO(this-> get_logger(), msg.data.c_str());
			RCLCPP_WARN(this-> get_logger(), msg.data.c_str());
			RCLCPP_DEBUG(this-> get_logger(), msg.data.c_str());
			RCLCPP_ERROR(this-> get_logger(), msg.data.c_str());
```
## DEBUG sehen
```shell
$ ros2 run package1_name exe1_name --ros-args --log-level debug
```
### node spezifizieren
```shell
$ ros2 run package1_name exe1_name --ros-args --log-level hello_world_sub_node:=debug
```

# Ros Message Types / Interfaces
- Beachten: in package xml file und cmake includen
- XML File:
    - depends
- CMake:
    - find_package
    - ament_target_dependencies

## ROS Befehl mit Interfaces
```shell
# Was ist an Interfaces für uns verfügbar
$ ros2 interface list
```
## STD_MSG
- Die genauen Typen sind unter: https://index.ros.org/p/std_msgs/ zu finden
- Als Header inlcuden

## SENSOR_MSGS
- von Sensoren 
- Link: https://index.ros.org/p/sensor_msgs/


# ROS Parameters

## Parameter-Kommandos
```shell
# Parameterliste zeigen
$ ros2 param list
# Getter von der Node
$ ros2 param get /node <parameter>
# Setter für den Node
$ ros2 param set /node <parameter>
```

## Parameter deklarieren und Parameter-getter in einer Methode 

### 1.Möglichkeit über eine Variable

```cpp
const float RPM_DEFAULT_VALUE = 10.0;
this->declare_parameter<float>("rpm_val", RPM_DEFAULT_VALUE);

// In der Pub-Methode
void publish_rpm_value(){
    //...
    this->get_parameter("rpm_val", rpm_val_param);
    message.data  = rpm_val_param;
    publisher_->publish(message);
}

float rpm_val_param = RPM_DEFAULT_VALUE;
```

### 2.Möglichkeit über ein Objekt


```cpp
const float RPM_DEFAULT_VALUE = 10.0;
this->declare_parameter<float>("rpm_val", RPM_DEFAULT_VALUE);

// In der Pub-Methode
void publish_rpm_value(){
    //...
    rclcpp::Parameter param_obj = this->get_parameter("rpm_val");
    message.data  = rpm_val_param_obj.as_double();
    publisher_->publish(message);
}

// Setter gibt es auch 
this->set_parameter(rclcpp::Parameter("your_param_name", 5.0));
```

# Launch Dateien
- Starte mehrere Nodes und konfiguiere sie alle über den Launch File

## Schritte
1. Launch Ordner im Ros Package erstellen
2. Launch Python Datei erstellen
    <name>.launch.py
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
def generate_launch_description():
return LaunchDescription([
Node(
package="your_package_name",
executable="your_node_exe_name",
name="node_name_override",
parameters=[
{"your_param": 5.0}
]
),
ExecuteProcess(
cmd=['ros2', 'topic', 'echo', '/topic_name'],
output='screen'
)
])
```
3. Launch File muss auch gefunden werden in der CMAKE List
```cmake
install(DIRECTORY
launch
DESTINATION share/${PROJECT_NAME}/
)
```

### Terminal Komando
```shell
ros2 launch your_package_name your_launch_file.py
```

# Ros 2 Interfaces

## Custom Interface
1. Ordner anlegen mit srv
2. Dependencies im package.xml rein
3. CMAKE List anpassen
4. COLCON BUILD

### Wo findet man diesn Service?
- Im Install Ordner -> include Ordner
- /home/ubuntu/Desktop/learn/Workspaces/ros2_cpp_ws/install/udemy_ros2_pkg/include/**

## Terminal Komando
```shell
# Interfaces auflisten
ros2 interface list
# Spezifisch
ros2 interface show <interface> 
# Interface laufen lassen
ros2 run <package> service_server
# Serviceaufruf durch Client
ros2 service call /package <parameter>
```



-----------------------------

# Docs

## Launch File -> Node.py
- Link: https://github.com/ros2/launch_ros/blob/humble/launch_ros/launch_ros/actions/node.py

## Colcon
- Link: https://colcon.readthedocs.io/en/released/user/quick-start.html

## Ros2 C++ Client Libary API
- Link: https://docs.ros2.org/foxy/api/rclcpp/index.html

## Ros STD_MSG
- Link: https://index.ros.org/p/std_msgs/

## ROS Logs
- Link: https://docs.ros.org/en/humble/Tutorials/Demos/Logging-and-logger-configuration.html

## ROS Interfaces
- Link: https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html

## ROS1 Cheat Sheet
- Link: https://mirror.umd.edu/roswiki/attachments/de/ROScheatsheet.pdf
- Link: https://www.generationrobots.com/media/ROS_Cheat_Sheet_Melodic.pdf
- Link: https://w3.cs.jmu.edu/spragunr/CS354_S19/handouts/ROSCheatsheet.pdf
- Link: https://kapeli.com/cheat_sheets/ROS.docset/Contents/Resources/Documents/index


# GitHub
## Add SSH to your Computer for Accessing GitHub
- Link: https://www.youtube.com/watch?v=wScfLiE1oTY
### Wenn nicht, hier die Steps:
```shell
1. $ ls -al ~/.ssh
# Lists the files in your .ssh directory, if they exist
2. $ for key in ~/.ssh/id_*; do ssh-keygen -l -f "${key}"; done | uniq
3. $ ssh-keygen -t ed25519 -C "your_email@example.com"
4. $ cd ~/.ssh cat id_ed25519.pub 
# Public Key kopieren
5. # Github Acc SSH Key hinzufügen
6. # Gehe zum Github Repo
7. $ git remote -v
8. # GitHub SSH URL kopieren
9. $ git remote set-url origin <GitHub SSH URL>
10. $ eval $(shh-agent)
11. $ ssh-add -k ~/.ssh/id_ed25519
12. $ git pull
```



