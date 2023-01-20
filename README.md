# ROS2 Template Programs(開発中)
ROS2のトピック通信、サービス通信、アクションのテンプレートプログラム

# Topic
std_msgs/String型のmessageをtopic通信

## C++
```bash
$ ros2 launch ros2_template_programs topic_template.xml
```
- [topic_template.xml](launch/topic_template.launch)
- [topic_publisher_template.cpp](src/topic_publisher_template.cpp)
- [topic_subscriber_template.cpp](src/topic_subscriber_template.cpp)

## Python
```py
$ ros2 launch ros2_template_programs topic_template_py.xml
```
- [topic_template_py.xml](launch/topic_template_py.xml)
- [topic_publisher_template.py](scripts/topic_publisher_template.py)
- [topic_subscriber_template.py](scripts/topic_subscriber_template.py)

<div align="center">
    <img src="doc/img/topic.png">
</div>

# 補足
## パッケージの作成方法
```py
# ament_cmake
$ ros2 pkg create package_name --build-type ament_cmake --dependencies rclcpp rclpy std_msgs
# ament_python
$ ros2 pkg create package_name --build-type ament_python --dependencies rclcpp rclpy std_msgs
```

## ビルド
```py
# 全パッケージのビルド
$ colcon build
# 指定パッケージのビルド
$ colcon build　--symlink-install --packages-up-to package_name
```
- `--symlink-install`
    - 可能な限りリンクを使用して2重にファイルをつくらない？
- `--packages-up-to [package1 package2 ...]`
    - 指定されたパッケージとそれに依存関係のあるパッケージのみをビルドする．
    - 複数指定する場合にはスペース区切りとする．

# Reference
- [Writing a simple publisher and subscriber (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html)
- [Using parameters in a class (Python)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Using-Parameters-In-A-Class-Python.html)
- [ROS2でyamlファイルからパラメータを設定する。](https://qiita.com/shigeharu_shibahata/items/82e8f562d2e6395ba115)
- [ROS2プロジェクトの作成](https://qiita.com/NeK/items/1d13d41bd0565e8da854)
- [メタビルドシステムament](https://www.youtalk.jp/2017/05/29/ament.html)
- [[ROS2 foxy] c++とpython共存パッケージのテンプレート](https://qiita.com/ousagi_sama/items/e1eb921f1b2e6b890133)
- [Create a ROS2 package for Both Python and Cpp Nodes](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)
- [Creating a launch file](https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html)
- [ROS 2 EloquentのXML記法を使ったLaunchシステム](https://www.youtalk.jp/2019/12/06/launch-xml.html)
- [ROS2プログラミング入門 #8 ノードをクラスにする](https://zenn.dev/uchidaryo/articles/ros2-programming-8)