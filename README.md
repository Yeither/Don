# Don算法
大创
## 文件树

.<br>
├── don<br>
│&emsp;&emsp;├── don.cpp<br>
│&emsp;&emsp;└── don.h<br>
├── lib.h<br>
├── main.cpp<br>
├── pointcloud<br>
├── readme.md<br>
├── setting.yml<br>
├── tools.hpp<br>
└── view<br>
&emsp;&emsp;├── view.cpp<br>
&emsp;&emsp;└── view.h<br>

don:don相关算法<br>
lib.h:用到的库<br>
pointcloud:储存点云的文件夹<br>
setting.yml:相关参数设置<br>
tools.hpp:一些其他函数<br>
view：可视化相关函数<br>
<br>
## setting.yml
  SAVE:&emsp;&emsp;&emsp;&emsp;&ensp;#bool,是否储存点云<br>
  Get_Nomal:&emsp;&emsp;#bool,是否计算每一个点的法向量，更改为别的聚类时可以用，有点耗时，先禁了<br>
  VoxelGrid_filter: #bool,false为在don前滤波，true为在don后ransac前滤波<br>
  scale1:&emsp;&emsp;&emsp;&emsp;#double<br>
  scale2:&emsp;&emsp;&emsp;&emsp;#double,大型点云中scale最后结果0.8，8效果较好<br>
  segradius:&emsp;&emsp;&emsp;#double,分割半径的缩放因子<br>
  leaf_size:&emsp;&emsp;&emsp;#滤波体素<br>
  address:&emsp;&emsp;&emsp;#点云地址<br>
  save_address:&emsp;#储存点云的地址<br>
