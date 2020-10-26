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


## 简单说说关于机器人的那些事儿
机器人无非就是电脑通过软件控制硬件行动，把电脑和各种硬件连起来并不难，难点往往在基础的物理、电路知识，和指挥硬件“如何运动”的数学策略。这里且不展开描述那些难点，你现在需要知道的就是，我们马上要做的事情，完全不会涉及到它们。

我们现在要做的事情，是把设计好的硬件用电线或者无线信号连起来，然后通过电脑向它们发出一些简单的指令，让硬件“听懂”我们的指令并且乖乖动起来！也许你现在觉得电脑能让硬件动起来真神奇（我也这么想），但其实最早发明电脑的先辈们都是先用电脑控制硬件，之后花了好长时间才终于造出我们现在熟悉的高级计算机，反而我们居然不需要弄懂计算机里面的原理，就可以直接在软件层面使用了！说实话，老人们也许会觉得我们这样子才是真的不可思议。（这里其实涉及到了很本质的计算机的思想，且不展开多说了，如果有需要，可以联系我搞个小课堂什么的）

所以，你不妨长出一口气，小菜一碟~

我们的大致步骤是：购买硬件 => 尝试用你的电脑和狗狗的电脑通话 => 尝试用狗狗的电脑指挥硬件 => 把硬件组装起来 => 让狗狗动起来！

下面马上开始！

## 硬件准备
硬件包括3D打印文件和其它可以直接购买到的东西，3D打印需要你联系3D打印店家，把相应的文件发过去打印。

硬件所需的全部物料清单都在“硬件部分”文件夹里，里面有详细的介绍。这里建议你（除非本身比较懂）最好和我用完全一样的硬件，因为不同的硬件之间合作的方式会有差异，为了避免不必要的差异，和我保持一致是最安全的。


## 软件上手
软件主要由三部分组成：测试、校准、运行。

全部代码都在“软件部分”文件夹里，里面有详细的介绍。我同样建议你和我用同样的系统、软件，因为不同的软件之间合作的方式也有差异，不同的系统很可能也内置了不同的软件，导致同样的指令没办法起到相同的效果。为了避免不必要的差异，和我保持一致是最安全的。


## 关于中文攻略的作者

嗨朋友，你好！我是李必昂（Beyond），本科毕业于香港城市大学商学院，主修经济学、辅修计算机科学，目前于香港中文大学工学院修读MPhil学位，研究下肢外骨骼。

我不久前才刚刚从商科转到机器人方向。作为一名高龄入门工程师，如果你（的孩子）恰好也是新手正在入门工程，我可太理解你面对繁杂的工程知识时心里的慌张和焦躁了。但是我也明白，工程的组成细胞总是简洁的，它的繁杂在于大量细碎零件的堆砌，导致常常一入门就必须面对已经堆成小丘的一大堆困难。因此，每每回顾 Spot Micro 四足机器人的开源社区，我都心怀感激，感谢他们为我这样的新手搭了一把入门的梯子，它复杂到可以涵盖机器人的基本思想、同时又简洁到不至于身陷过多的琐碎细节。致敬开源。

同时，作为一个来自商学院的转业新手，也许我可以贡献一些来自商科生的表达能力、和来自高龄入门选手的更成熟的入门体会。

此外，如果你想交个朋友，或者如果你需要更私人一些的援助（比如物料采购、远程教学、面对面教学···），也欢迎你联系我，直接加我微信就好~

微信号：fading_fire_

（请备注：中文姓名-机构-领域，比如：李必昂-港中文-机器人）

邮箱：bianglibeyond@link.cuhk.edu.hk


## 拓展链接
这部分没有翻译，是因为如果你现在看不懂，那你就完全没有必要看下去了，往前就是高端玩家的世界了哈哈哈。

Spot Micro AI community: https://gitlab.com/custom_robots/spotmicroai

Research paper used for inverse kinematics:
`Sen, Muhammed Arif & Bakircioglu, Veli & Kalyoncu, Mete. (2017).
Inverse Kinematic Analysis Of A Quadruped Robot.
International Journal of Scientific & Technology Research. 6.`

Stanford robotics for inspiration for gait code: https://github.com/stanfordroboticsclub/StanfordQuadruped
