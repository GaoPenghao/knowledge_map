本文主要记录在使用Apollo docker时遇到的环境管理的问题，并记录对docker环境的修改。

# apt-get update出错

在执行`sudo apt-get update`是出现许可报错，按照博客的指示https://blog.csdn.net/Chaowanq/article/details/121559709进行修复，重新安装许可，该问题解决。

# 使用matplotlib画图出错

安装以下依赖，可以解决

```bash
sudo apt-get update
sudo apt-get install tcl-dev tk-dev python3-tk
```





