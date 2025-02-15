# Readme.md
## 文件树

.<br>
├── don<br>
│$~~~~~$├── don.cpp<br>
│$~~~~~$└── don.h<br>
├── lib.h<br>
├── main.cpp<br>
├── pointcloud<br>
├── readme.md<br>
├── setting.yml<br>
├── tools.hpp<br>
└── view<br>
$~~~~~~~$├── view.cpp<br>
$~~~~~~~$└── view.h<br>

don:don相关算法<br>
lib.h:用到的库<br>
pointcloud:储存点云的文件夹<br>
setting.yml:相关参数设置<br>
tools.hpp:一些其他函数<br>
view：可视化相关函数<br>
<br>
## setting.yml
  SAVE:$~~~~~~~~~~~~~~~~$#bool,是否储存点云<br>
  Get_Nomal:$~~~~~~~$#bool,是否计算每一个点的法向量，更改为别的聚类时可以用，有点耗时，先禁了<br>
  VoxelGrid_filter: #bool,false为在don前滤波，true为在don后ransac前滤波<br>
  scale1:$~~~~~~~~~~~~~~~$#double<br>
  scale2:$~~~~~~~~~~~~~~~$#double,大型点云中scale最后结果0.8，8效果较好<br>
  segradius:$~~~~~~~~~~$#double,分割半径的缩放因子<br>
  leaf_size:$~~~~~~~~~~~$#滤波体素<br>
  address:$~~~~~~~~~~~~~$#点云地址<br>
  save_address:$~~~~~$#储存点云的地址<br>

