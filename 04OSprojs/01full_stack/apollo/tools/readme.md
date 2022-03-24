本文用于记录在Apollo开发过程中常用的一些工具

# Cyber tools

cyber tools的源码目录为`cyber/tools`

## cyber_recorder

查看常用的命令选项

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder -h
usage: cyber_recorder -h [options]
Unknown command: -h
usage: cyber_recorder <command> [<args>]
The cyber_recorder commands are:
	info	Show information of an exist record.
	play	Play an exist record.
	record	Record same topic.
	split	Split an exist record.
	recover	Recover an exist record.
```

可以看到，-h并不是其命令选项，这里常用的命令选项有info\play\recorder\split\recover五种命令选项，后面将一一示例。

### info

info为查看包的基本信息，查看info的使用方法

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder info -h
usage: cyber_recorder info file
usage: cyber_recorder info [options]
	-h, --help				show help message
```

使用info查看包的信息

```bash
cyber_recorder info docs/demo_guide/demo_3.5.record
```

### play

play为播放一个包，查看play的使用方法

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder play -h
usage: cyber_recorder play [options]
	-f, --file <file>			input record file
	-a, --all				play all
	-c, --white-channel <name>		only play the specified channel
	-k, --black-channel <name>		not play the specified channel
	-l, --loop				loop play
	-r, --rate <1.0>			multiply the play rate by FACTOR
	-b, --begin 2018-07-01-00:00:00	play the record begin at
	-e, --end 2018-07-01-00:01:00		play the record end at
	-s, --start <seconds>			play started at n seconds
	-d, --delay <seconds>			play delayed n seconds
	-p, --preload <seconds>			play after trying to preload n second(s)
	-h, --help				show help message

```

使用play播放包的示例如下

```bash
cyber_recorder play -f docs/demo_guide/demo_3.5.record -l
```

### record

record为录制一个包

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder record -h
usage: cyber_recorder record [options]
	-o, --output <file>			output record file
	-a, --all				record all
	-c, --white-channel <name>		only record the specified channel
	-k, --black-channel <name>		not record the specified channel
	-i, --segment-interval <seconds>	record segmented every n second(s)
	-m, --segment-size <MB>			record segmented every n megabyte(s)
	-h, --help				show help message

```

-a 为录制所有通道

```bash
cyber_recorder record -a
```

 -c 为指定录制的通道

```bash
cyber_recorder record -c /apollo/planning
```

-k 为指定不录制的通道

```bash
cyber_recorder record -k /apollo/planning
```

### split

split为从一个包中分离出另外一个包，并指定需要提取的通道名称

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder split -h
usage: cyber_recorder split [options]
	-f, --file <file>			input record file
	-o, --output <file>			output record file
	-c, --white-channel <name>		only split the specified channel
	-k, --black-channel <name>		not split the specified channel
	-b, --begin 2018-07-01-00:00:00	split the record begin at
	-e, --end 2018-07-01-00:01:00		split the record end at
	-h, --help				show help message
```

具体用法示例

```bash
cyber_recorder split -f docs/demo_guide/demo_3.5.record -o data/bag/gph.record -k /apollo/planning
```

### recover

recover为修复某个已存在的包

```bash
[zy@in-dev-docker:/apollo]$ cyber_recorder recover -h
usage: cyber_recorder recover [options]
	-f, --file <file>			input record file
	-o, --output <file>			output record file
	-h, --help				show help message
```

## cyber_monitor

使用cyber_monitor进行通道信息的监控，其中主要包括命令选项-c和一些交互快捷键，其中重点熟悉==f, t, i, b, n, m以及 ,==的使用

```bash
[zy@in-dev-docker:/apollo]$ cyber_monitor -h
Usage:
cyber_monitor  [option]
Option:
   -h print help info
   -c specify one channel
Interactive Command:
Common Commands for all:
   q | Q | Esc -- quit
   Backspace -- go back
   h | H -- go to show help info

Common Commands for Topology and Channel Message:
   PgDn | ^d -- show next page
   PgUp | ^u -- show previous page

   Up Arrow -- move up one line
   Down Arrow -- move down one line
   Right Arrow -- enter the selected Channel or Repeated Datum
   Left Arrow -- go back to the upper level

   Enter -- the same with Right Arrow key
   a | A -- the same with Left Arrow key
   d | D -- the same with Right Arrow key
   w | W -- the same with Up Arrow key
   s | S -- the same with Down Arrow key

Commands for Topology message:
   f | F -- show frame ratio for all channel messages
   t | T -- show channel message type

   Space -- Enable|Disable channel Message

Commands for Channel:
   i | I -- show Reader and Writers of Channel
   b | B -- show Debug String of Channel Message

Commands for Channel Repeated Datum:
   n | N -- next repeated data item
   m | M -- previous repeated data item
   , -- enable|disable to show all repeated items

```

使用`-c`监控特定的通道

```bash
cyber_monitor -c /apollo/planning
```

直接运行`cyber_monitor`可以监控所有的通道，其中红色代表未被激活的通道，绿色代表被激活的通道。

## cyber_channel

等同于`ros_topic`，其使用方法如下所示

```bash
[zy@in-dev-docker:/apollo]$ cyber_channel -h
cyber_channel is a command-line tool for printing information about CyberRT Channels.

Commands:
	cyber_channel list	list active channels
	cyber_channel info	print information about active channel
	cyber_channel echo	print messages to screen
	cyber_channel hz	display publishing rate of channel
	cyber_channel bw	display bandwidth used by channel
	cyber_channel type	print channel type

Type cyber_channel <command> -h for more detailed usage, e.g. 'cyber_channel echo -h'
```

### list

打印出所有被激活的通道

```bash
cyber_channel list
```

### info

显式通道信息

```bash	
cyber_channel info /apollo/planning   # 显示指定通道的信息

cyber_channel info -a                 # 显示所有通道的信息
```

### echo

打印通道信息

```bash
cyber_channel echo /apollo/planning
```

### hz

显示通道的发布频率

```bash
cyber_channel hz /apollo/planning
```

### bw

显示通道的带宽（读写速度KB/s）

```bash
cyber_channel bw /apollo/planning
```

### type

显示通道的消息类型

```bash
cyber_channel type /apollo/planning
```

## cyber_launch

### start

启动一个launch文件

```bash
cyber_launch start [launch_file]
```

### stop

停止一个launch文件，默认停止所有的launch文件

```bash
cyber_launch stop [launch_file]
```

## cyber_node

### list

显示激活的node

```bash
cyber_node list
```

### info

显示node的信息

```bash
cyber_node info
```

## cyber_services

```bash
[zy@in-dev-docker:/apollo]$ cyber_service -h
cyber_service is a command-line tool for printing information about CyberRT Services.

Commands:
	cyber_service list	list active services
	cyber_service info	print information about active service

Type cyber_service <command> -h for more detailed usage, e.g. 'cyber_service info -h'
```

### list

显示激活的服务

```bash
cyber_service list 
```

### info

显示服务的信息

```bash
cyber_service info [service]
```







