# Spot Micro 四足机器人中文攻略

![Spot Micro Walking](assets/spot_micro_walking.gif)

视频来源（打不开没关系，它不是很重要）: https://www.youtube.com/watch?v=S-uzWG9Z-5E


致谢/Credit to:

我们的所有代码来自于Mike的开源仓库：https://github.com/mike4192/spotMicro

Our codes are from Mike's repository

模型原型来自于 Spot Micro AI 社区：https://gitlab.com/custom_robots/spotmicroai

Our models are based from Spot Micro AI Community

这份中文攻略的主要工作是把英文开源社区的内容翻译成中文、把整理得更好读一些。

Our contribution is more about tranlating those great open-sourced technical knowledge into Chinese with a more beginner-friendly mind map.


下面马上开始！

## Overview
This project is the source code for a Spot Micro quadruped, a 4 legged open source robot. This code is capable of keyboard control of a spot micro robot with sit, stand, angle command, and walk capability. The software is implemented on a Raspberry Pi 3B computer running Ubuntu 16.04.

The software is composed ot C++ and python nodes in a ROS framework.




## 关于中文翻攻略的作者

嗨朋友，你好！我是李必昂（Beyond），本科毕业于香港城市大学商学院，主修经济学、辅修计算机科学，目前于香港中文大学工学院修读MPhil学位，研究下肢外骨骼。

我不久前才刚刚从商科转到机器人方向。作为一名高龄入门工程师，如果你（的孩子）恰好也是新手正在入门工程，我可太理解你面对繁杂的工程知识时心里的慌张和焦躁了。但是我也明白，工程的组成细胞总是简洁的，它的繁杂在于大量细碎零件的堆砌，导致常常一入门就必须面对已经堆成小丘的一大堆困难。因此，每每回顾 Spot Micro 四足机器人的开源社区，我都心怀感激，感谢他们为我这样的新手搭了一把入门的梯子，它复杂到可以涵盖机器人的基本思想、同时又简洁到不至于身陷过多的琐碎细节。致敬开源。

同时，作为一个来自商学院的转业新手，也许我可以贡献一些来自商科生的表达能力、和来自高龄入门选手的更成熟的入门体会。

此外，如果你想交个朋友，或者如果你需要更私人一些的援助（比如物料采购、远程教学、面对面教学···），也欢迎你联系我，直接加我微信就好~

微信号：fading_fire_

邮箱：bianglibeyond@link.cuhk.edu.hk


## 拓展链接
这部分没有翻译，是因为如果你现在看不懂，那你就完全没有必要看下去了，往前就是高端玩家的世界了哈哈哈。

Spot Micro AI community: https://gitlab.com/custom_robots/spotmicroai

Research paper used for inverse kinematics:
`Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017).
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6.`

Stanford robotics for inspiration for gait code: https://github.com/stanfordroboticsclub/StanfordQuadruped
