# hdl_graph_slam

[![Build](https://github.com/koide3/hdl_graph_slam/actions/workflows/build.yml/badge.svg)](https://github.com/koide3/hdl_graph_slam/actions/workflows/build.yml) on Melodic & Noetic

*目次*  
1. [システム概要](##システム概要)  
  
2. [準備](##準備)  
  
3. [システム利用手順](##システム利用手順)  
  
4. [動作確認](##動作確認)  

5. [関連リンク](##関連リンク)  


## システム概要
***hdl_graph_slam*** は3D LIDARを使用したリアルタイム6DOF SLAM用のオープンソースROSパッケージです。NDTスキャンマッチングに基づくオドメトリ推定とループ検出による3DグラフSLAMがベースとなります。また、GPS、IMU加速度 (重力ベクトル)，IMU姿勢 (磁器センサー)、床面 (点群での検出)など、複数のグラフ制約をサポートしています。このパッケージは、Velodyne社 (HDL32e、VLP16)およびRoboSense社 (16ch)のセンサーと屋内外の環境でテストされています。

<img src="imgs/hdl_graph_slam.png" width="712pix" />


### 1.1 システム構成

#### 1.1.1 Nodelets
***hdl_graph_slam*** は4つのNodeletsから構成されます。

- *prefiltering_nodelet*
- *scan_matching_odometry_nodelet*
- *floor_detection_nodelet*
- *hdl_graph_slam_nodelet*

入力された点群は、はじめに*prefiltering_nodelet*によってダウンサンプルされ、次のnodeletsに渡されます。*scan_matching_odometry_nodelet*が連続するフレーム間のスキャンマッチングを繰り返し適用してセンサーの姿勢を推定する(オドメトリ推定)のに対し、*floor_detection_nodelet*はRANSACにより床面を検出します。推定されたオドメトリと検出された床面は*hdl_graph_slam*に送られます。スキャンマッチングの累積誤差を補正するために、ループ検出を行い、様々な制約を考慮したポーズグラフを最適化します。

<img src="imgs/nodelets.png" width="712pix" />

## 1.1.2 Constraints (Edges)
roslaunchファイルのparamsを変更することで、各制約の有効/無効を切り替えられるほか、各制約の重み(\*_stddev)とロバストカーネル(\*_robust_kernel)を変更することも可能です。

- ***Odometry***

- ***Loop closure***

- ***GPS***
  - */gps/geopoint* (geographic_msgs/GeoPointStamped)
  - */gps/navsat* (sensor_msgs/NavSatFix)
  - */gpsimu_driver/nmea_sentence* (nmea_msgs/Sentence)

hdl_graph_slamはいくつかのGPSのメッセージをサポートしています。サポートされるすべてのタイプは、緯度、経度、高度を含んでいます。hdl_graph_slamはそれらを[the UTM coordinate](http://wiki.ros.org/geodesy)に変換し、3次元位置制約としてグラフに追加します。もし高度をNaNに設定した場合、GPSデータは2次元の座標として扱われます。GeoPointは最も基本的なもので、lat、lon、altのみで構成されています。NavSatFixは多くの情報を提供していますが、ここではlat、lon、altのみを使用し、他のデータは無視します。HDL32eを使用している場合、*/gpsimu_driver/nmea_sentence*を介して*hdl_graph_slam*と*velodyne_driver*を直接接続することが可能です。

- ***IMU acceleration (gravity vector)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

この制約により各ポーズノードは、そのノードに関連する加速度ベクトルが垂直になるように（重力ベクトルとして）回転されます。これはスキャンマッチングの累積チルト回転誤差を補正するのに有効です。センサーの動きによる加速度は無視するため、この制約に大きなウェイトを与えるべきではありません．

- ***IMU orientation (magnetic sensor)***
  - */gpsimu_driver/imu_data* (sensor_msgs/Imu)

IMUに信頼性の高い磁気姿勢センサーが搭載されている場合、3D回転制約として姿勢データをグラフに追加することができます。ただし磁気姿勢センサーは、外部の磁気の影響を受けることがあります。そのような場合は、この制約を無効にする必要があります。

- ***Floor plane***
  - */floor_detection/floor_coeffs* (hdl_graph_slam/FloorCoeffs)

この制約により、姿勢ノードの床面 (RANSACで検出)が同じになるようにグラフを最適化します。これは広い平坦な屋内環境において、スキャンマッチングの累積回転誤差を補償するために設計されています。

### 1.1.3 ROS
- ROSパラメータ  
設定可能なROSパラメータは、*launch/hdl_graph_slam.launch*にrosparamsとしてリストアップされています。

- ROSサービス  
以下が設定されています。
　- */hdl_graph_slam/dump*  (hdl_graph_slam/DumpGraph)
 　- フォルダに内部データ(point clouds, floor coeffs, odoms, and pose graph)を全て保存する。
　- */hdl_graph_slam/save_map*  (hdl_graph_slam/SaveMap)
 　- PCD形式で生成した地図を保存する。

## 2. 環境構築 
### 2.1 動作環境
***hdl_graph_slam***は,
Docker(hdl_graph_slma用)とLocal環境のROS(rviz用)で動作させることを想定しています。  
(Local環境のみでも動作は可能です。)

| Component | Requirement |
| :-- | :-- |
| OS | Ubuntu 18.04LTS, 20.04LTS |
| Python | Versions 3.8 |

ROSのインストールは下記を参照してください。
- [Melodic (Ubuntu18.04LTSの場合)](https://wiki.ros.org/melodic)
- [Noetic (Ubuntu20.04LTSの場合)](http://wiki.ros.org/noetic)


### 2.2 依存関係
***hdl_graph_slam***は以下のライブラリとROSパッケージが必要です。

ライブラリ
- OpenMP
- PCL
- g2o
- suitesparse

ROSパッケージ
- geodesy
- nmea_msgs
- pcl_ros
- [ndt_omp](https://github.com/koide3/ndt_omp)
- [fast_gicp](https://github.com/SMRT-AIST/fast_gicp)


### 2.3 セットアップ
ROS Melodicの場合 (Local環境にROSをインストール後)
~~~
sudo apt-get install ros-melodic-geodesy ros-melodic-pcl-ros ros-melodic-nmea-msgs ros-melodic-libg2o
cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git -b melodic
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
~~~

ROS Noeticの場合 (Local環境にROSをインストール後)
~~~
sudo apt-get install ros-noetic-geodesy ros-noetic-pcl-ros ros-noetic-nmea-msgs ros-noetic-libg2o

cd catkin_ws/src
git clone https://github.com/koide3/ndt_omp.git
git clone https://github.com/SMRT-AIST/fast_gicp.git --recursive
git clone https://github.com/koide3/hdl_graph_slam

cd .. && catkin_make -DCMAKE_BUILD_TYPE=Release
~~~

**[optional]** *bag_player.py* scriptはProgressBar2を必要とします。
~~~
sudo pip install ProgressBar2
~~~


#### 実行手順
1. 好きなフォルダにこのリポジトリをクローン
~~~
git clone https://github.com/koide3/hdl_graph_slam
~~~

2. Dockerイメージのビルド
~~~ 
cd hdl_graph_slam/docker  
./build.sh
~~~

3. roscoreの起動
~~~
# Local PC (Terminal 1)
roscore
~~~

4. Local環境でrvizの起動
~~~
# Local PC (Terminal 2、Local環境)
rosparam set use_sim_time true

cd hdl_graph_slam/rviz  
rviz -d hdl_graph_slam.rviz
~~~

5. rosbagの再生
~~~
# Local PC (Terminal 3、Local環境)
wget http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz
tar -zxvf hdl_400.bag.tar.gz
rosbag play --clock hdl_400.bag
~~~

6. Dockerコンテナを起動し、hdl_graph_slmaの起動
~~~
# Local PC (Terminal 4, Docker)
cd hdl_graph_slam/docker
./run.sh
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
~~~

Rvizでhdl_graph_slamが起動していることを確認できます。  
![hdl_graph_slam](https://user-images.githubusercontent.com/31344317/98347836-4fed5a00-205b-11eb-931c-158f6cd056bf.gif)


## 3. システム利用手順
hdl_graph_slamを利用するために、場面に応じたrosbagファイルをいくつか提供しています。  
ご参考にください。

### 3.1 例1 屋内環境
Rosbag file (recorded in a small room):

- [hdl_501.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501.bag.tar.gz) (raw data, 344MB)
- [hdl_501_filtered.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501_filtered.bag.tar.gz) (downsampled data, 57MB, **Recommended!**)

1. roscore
~~~
(Terminal 1, Local環境)
roscore
~~~

2. Dockerコンテナの起動
~~~
(Terminal 2、Docker)
cd hdl_graph_slam/docker
./run.sh

rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_501.launch
~~~

3. rviz
~~~
(Terminal 3, Local環境)
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
~~~

4. rosbag再生
~~~
(Terminal 4, Local環境)
wget http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_501_filtered.bag.tar.gz
tar -zxvf hdl_501_filtered.bag.tar.gz
rosbag play --clock hdl_501_filtered.bag

(また再生速度を自動的に調整し、可能な限り高速にデータを処理するbag_player.pyも提供しています。)
rosrun hdl_graph_slam bag_player.py hdl_501_filtered.bag
~~~

5. 地図の保存
以下のコマンドで、生成したマップを保存できます。
root権限でmapは保存されるため、chmod 777 ./map.pcdで権限を変更できます。
~~~
(Terminal 5, Docker)
docker exec -it [docker name] bash
(docker nameはdocker ps -aでNameから起動しているコンテナ名を確認できます。)

rosservice call /hdl_graph_slam/save_map "resolution: 0.05
destination: '/root/catkin_ws/src/map.pcd'"
~~~

上記のコマンドを入力すると、下記のようにrvizで実行している様子が確認できます。  
<img src="imgs/top.png" height="256pix" /> <img src="imgs/birds.png" height="256pix" />



### 3.2 屋外環境 
Bag file (recorded in an outdoor environment):
- [hdl_400.bag.tar.gz](http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz) (raw data, about 900MB)

1. roscore
~~~
(Terminal 1, Local環境)
roscore
~~~

2. Dockerコンテナの起動
~~~
(Terminal 2、Docker)
cd hdl_graph_slam/docker
./run.sh

rosparam set use_sim_time true
roslaunch hdl_graph_slam hdl_graph_slam_400.launch
~~~

3. rviz
~~~
(Terminal 3, Local環境)
roscd hdl_graph_slam/rviz
rviz -d hdl_graph_slam.rviz
~~~

4. rosbag再生
~~~
(Terminal 4, Local環境)
wget http://www.aisl.cs.tut.ac.jp/databases/hdl_graph_slam/hdl_400.bag.tar.gz
tar -zxvf hdl_400.bag.tar.gz
rosbag play --clock hdl_400.bag
~~~

上記のコマンドを入力すると、以下のように表示されます。  
地図保存の方法「屋内環境」のときに使用したコマンドと同様にできます。  
<img src="imgs/hdl_400_points.png" height="256pix" /> <img src="imgs/hdl_400_graph.png" height="256pix" />


### 3.3 GPSを使用した例
Ford Campus Vision and Lidar Data Set [\[URL\]](http://robots.engin.umich.edu/SoftwareData/Ford)

以下のスクリプトは、Ford Lidar Datasetをrosbagに変換して再生します。この例では、***hdl_graph_slam***がGPSデータを利用して、ポーズグラフを補正しています。

```bash
cd IJRR-Dataset-2
rosrun hdl_graph_slam ford2bag.py dataset-2.bag
rosrun hdl_graph_slam bag_player.py dataset-2.bag
```

<img src="imgs/ford1.png" height="200pix"/> <img src="imgs/ford2.png" height="200pix"/> <img src="imgs/ford3.png" height="200pix"/>

### 3.4 自身のシステムでhdl-graph-slamを使用する場合

1. static_transform_publisherを使用し、センサー (LIDAR，IMU，GPS)とシステムのbase_link間の変換を定義します (hdl_graph_slam.launchの行番号11を参照してください。)。すべてのセンサデータは共通のbase_linkフレームに変換され、SLAMアルゴリズムに供給されます．
 
2. 以下の設定で、***prefiltering_nodelet***の点群トピックをリマップします:

```bash
  <node pkg="nodelet" type="nodelet" name="prefiltering_nodelet" ...
    <remap from="/velodyne_points" to="/rslidar_points"/>
  ...
```

### 3.5 パラメータのチューニングについて

マッピングの品質はパラメータの設定に大きく依存します。特にスキャンマッチングパラメータは結果に大きな影響を与えます。以下の手順でパラメータをチューニングしてください。

- ***registration_method***
  **[updated]  
簡単には、ほとんどの場合はFAST_GICPを使用し、処理速度が重要な場合はFAST_VGICPかNDT_OMPを使うということです** このパラメータにより、オドメトリ推定とループ検出に使う登録方法を変更することができます。PCL1.7 (ROS kinetic)以前のGICPには、初期推測の処理にバグがあることに注意してください。**ROS kinecticまたはそれ以前のバージョンをお使いの場合は、GICPを使用しないでください**
  
- ***ndt_resolution***  
このパラメータはNDTのボクセルサイズを決定します。通常、屋外環境では大きな値が適しています(屋内では0.5 - 2.0 [m]、屋外では 2.0 - 10.0 [m])。NDTまたはNDT_OMPを選択した場合は、このパラメータを微調整して、良好なオドメトリ推定結果を得ることができます。

- ***他のパラメータ***  
設定可能なパラメータは、全て起動ファイルで確認できます。起動ファイルのテンプレート（屋内用hdl_graph_slam_501.launch、屋外用hdl_graph_slam_400.launch）をコピーして、起動ファイル内のパラメータを微調整してアプリケーションに適応させることができます。


## 4. 関連リンク  

### 4.1 ライセンス
このパッケージはBSD-2-Clause Licenseの下でリリースされています。
g2oに含まれるcholmodソルバーはGPLの下でライセンスされていることに注意してください。GPLを回避するために、cholmodに依存しないg2oをビルドする必要があるかもしれません。


### 4.2 関連パッケージ
- [interactive_slam](https://github.com/koide3/interactive_slam)
- [hdl_graph_slam](https://github.com/koide3/hdl_graph_slam)
- [hdl_localization](https://github.com/koide3/hdl_localization)
- [hdl_people_tracking](https://github.com/koide3/hdl_people_tracking)

<img src="imgs/packages.png"/>

### 4.3 関連論文
Kenji Koide, Jun Miura, and Emanuele Menegatti, A Portable 3D LIDAR-based System for Long-term and Wide-area People Behavior Measurement, Advanced Robotic Systems, 2019 [[link]](https://www.researchgate.net/publication/331283709_A_Portable_3D_LIDAR-based_System_for_Long-term_and_Wide-area_People_Behavior_Measurement).

### 4.4 連絡先
Kenji Koide, k.koide@aist.go.jp, https://staff.aist.go.jp/k.koide

Active Intelligent Systems Laboratory, Toyohashi University of Technology, Japan [\[URL\]](http://www.aisl.cs.tut.ac.jp)  
Mobile Robotics Research Team, National Institute of Advanced Industrial Science and Technology (AIST), Japan  [\[URL\]](https://unit.aist.go.jp/rirc/en/team/smart_mobility.html)
