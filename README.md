# RM2022--Reborn
RM2022工程超级对抗赛-Reborn

<img width="594" alt="image" src="https://user-images.githubusercontent.com/109509707/183019197-5a69a2ff-a9b2-4878-8a04-a31577332f6f.png">


1 软件方案设计

工程车要完成的主要任务：取矿、兑换点兑换、转矿和空接。
大资源岛和小资源岛高度不同， 因此分两种情况取矿， 取矿流程与机械设计相关， 最多一 次可以取三个矿石，通过气缸完成推出和夹取的动作，增加光电管判断旋转矿石和空接。利 用键盘按键和光电管判断改变工程车的状态， 工程车同时有几个状态，每个状态执行一套任务，互相不关联，一个状态改变，同时记录改变前后的两种状态，从而执行不同的流程。

<img width="217" alt="image" src="https://user-images.githubusercontent.com/109509707/183019927-14fcd312-4a10-434a-ac36-f9cb3361b059.png">
<img width="482" alt="image" src="https://user-images.githubusercontent.com/109509707/183019931-ba3dc0bd-9836-4c15-ab9e-4405c63a3122.png">

2 算法方案设计

2.1 PID 控制算法

串级控制， 就是采用两个控制器串联工作， 外环控制器的输出作为内环控制器的设定值， 由内环控制器的输出去操纵控制阀，从而对外环被控量具有更好的控制效果。
从串级控制的工作过程看来， 两个控制器是串联工作的， 以外环控制器为主导， 保证外环 主变量稳定为目的，两个控制器协调一致，互相配合。尤其是对于二次干扰，内环控制器首 先进行“粗调”，外环控制器再进一步“细调”，因此控制品质必然优于简单控制系统。
串级控制系统在结构上仅仅比简单控制系统多了一个内环回路， 可是实践证明， 对于相同 的干扰，串级控制系统的控制质量是简单控制系统无法比拟的。就外环回路看是一个定值控 制系统，但内环回路可看成是一个随动控制回路。外环控制器按负荷和操作条件的变化不断 纠正内环控制器的设定值，使内环控制器的设定值适应负荷和操作条件的变化。如果对象中 有较大非线性的部分包含到了内环回路中，则负荷和操作条件变化时，必然使内环回路的工 作点移动而影响其稳定性。但在串级结构中， 内环回路的变化对整个系统的稳定性影响很小， 所以从这个意义上说，串级控制系统能够适应不同负荷和操作条件的变化。

<img width="482" alt="image" src="https://user-images.githubusercontent.com/109509707/183019521-846af84e-3028-49cb-bf1f-d9bbb8041c04.png">


2.2 卡尔曼滤波算法

卡尔曼滤波五个等式

<img width="377" alt="image" src="https://user-images.githubusercontent.com/109509707/183019600-6269cc61-7f43-4341-a622-68506bd4d546.png">


算法设计分为两个函数创建卡尔曼滤波 kalmanCreate()和卡尔曼滤波计算 KalmanFilter()

 kalmanCreate 函数：
 
 <img width="336" alt="image" src="https://user-images.githubusercontent.com/109509707/183019609-5b6b9067-0ffa-43df-8b6c-70c4ea2f1cf7.png">

 KalmanFilter 函数：
 
 <img width="299" alt="image" src="https://user-images.githubusercontent.com/109509707/183019643-076b8727-b34c-4a06-b1b1-981f970e65de.png">

 卡尔曼滤波具体实现实例：
 
 <img width="493" alt="image" src="https://user-images.githubusercontent.com/109509707/183019653-71c87af4-dfa0-4d70-b482-ef7338dbe995.png">


3 调试过程 J-scope：用于数据可视化，辅助参数调整

 J-scope 界面图：
 
 <img width="490" alt="image" src="https://user-images.githubusercontent.com/109509707/183019732-b2201140-b7a4-4e53-87c2-32e093b3550c.png">


