﻿Field Programmable Robot (ver 0.1 alpha) by Yuri213212 at team "Team 16"
  Published at Digit Hackathon 2018

使用协议：对于所有文件遵循CC BY-NC-SA 4.0。
https://creativecommons.org/licenses/by-nc-sa/4.0/

此工程为本人在Digit Hackathon 2018参赛作品的担当部分，最终作品以队伍“チーム16”的名义提出，在大赛中获得了Filament奖。

最终成果为一辆玩具车，通过扫描放入的方块的方式实时写入移动程序，程序写入后可以多次执行。
本作品的初衷是给不会用电脑的小孩以玩玩具的方式学习编程基础。
由于颜色识别失败只能通过开关判断两种方块类型，目前的固件仅仅实现了前后左右移动命令的功能。
如果颜色识别模块（非本人负责）的数据足够好以至于能够分辨8种颜色，则可以考虑扩展命令集以实现对移动的精确时间控制以及增加循环、递归、条件判断等功能。

主要硬件（芯片）：
·单片机LPC1114FN28/102
·使用I2C通信的IO扩展芯片MCP23017
·使用I2C通信，基于SH1106的OLED屏幕
·4半H桥电机驱动芯片SN754410

外设：
·一个OLED屏幕用来显示内部状态
·16个命令指示LED用来提示正在执行的命令
·一个蜂鸣器用来提示执行状态
·两个电机用来提供小车的动力，需要电机驱动控制正转和反转
·两个按钮，其中一个是执行按钮，另一个是（硬件）重启按钮
·一个带颜色识别功能的输入模块（非本人负责）

执行效果：
·屏幕始终显示内部状态、命令计数以及命令内容
·命令计数为0时按下执行按钮，命令指示LED从后向前依次闪烁（提示输入命令）
·每输入一个命令，输入时蜂鸣器发出高频“哔”声，同时下一个命令指示LED被点亮
·命令计数不为0时按下执行按钮：
　·蜂鸣器先发出执行开始音效，同时全部命令指示LED熄灭
　·从前到后依次执行命令，同时点亮当前执行的命令对应的LED：
　　·如果命令有效则蜂鸣器发出低频“哔”声并执行命令对应的动作
　　·如果命令无效则蜂鸣器发错出错音效并暂停动作
　·所有命令执行完毕时，蜂鸣器发出执行结束音效，同时命令指示LED重新点亮从前到后和输入命令相同的数量
·按下重启按钮恢复至刚接通电源的状态
