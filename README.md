# ROS2 Template Programs(開発中)
ROS2のトピック通信、サービス通信、アクションのテンプレートプログラム

## パッケージの作成方法
```py
$ ros2 pkg create package_name --build-type ament_cmake --dependencies rclcpp
```
参考：[ROS2プロジェクトの作成](https://qiita.com/NeK/items/1d13d41bd0565e8da854)

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