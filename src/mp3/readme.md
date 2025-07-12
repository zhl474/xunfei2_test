
## 文字转语音的网站https://app.xunjiepdf.com/text2voice/
## 需要安装sox
 `sudo apt-get install sox`  `sudo apt-get install libsox-fmt-all` 

 
## 需要减小音量

`sox -v 0.1 voice_11.mp3 voice_1.mp3`


## 播放

`system("play XXX.mp3");`



 cp src/mp3/B_MAXmp3 ./
 chmod +x B_shuiguo.mp3  # 添加可执行权限（非必要）
 play B.mp3