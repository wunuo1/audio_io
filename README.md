# 功能介绍

audio_io功能包用于音频处理，ASR使用SenseVoice，[TTS](https://forum.d-robotics.cc/t/topic/33355)以及[KWS](https://forum.d-robotics.cc/t/topic/33616)使用[sherpa-onnx](https://github.com/k2-fsa/sherpa-onnx)，实现以下两条链路：
- 麦克风获取   --> ASR+KWS --> 发布语音识别内容
- 接收文字内容 -->   TTS   --> 扬声器输出

# 使用方法

## 准备工作

在体验之前，需要具备以下基本条件：

- RDK X5已烧录好Ubuntu 22.04系统镜像
- 确保MagicBox的麦克风扬声器能够正常使用
- 或者USB音响正确连接到RDK X5的usb接口、音频板正确连接到RDK X5的3.5mm的耳麦接口


## 运行智能语音程序

启动audio_io package：

`ros2 launch audio_io audio_io.launch.py`

# 接口说明
## 话题

| 名称    | 消息类型                                                   | 说明                               |
| ------------ | -----------------------------------------------------| ---------------------------------- |
| /prompt_text | std_msgs/msg/String                                  | 发布ASR识别结果，该名称与大模型输入话题一致 |
| /tts_text    | std_msgs/msg/String                                  | 接收要TTS的内容，与大模型结果发布话题一致   |

## 参数

| 参数名               | 类型        | 解释               | 是否必须 | 支持的配置       | 默认值       |
| -------------------- | ----------- | ------------------ | -------- | ---------------- | ------------ |
| micphone_name          | std::string | 语音采集设备       | 否       | 根据实际情况配置 | plughw:0,0     |
| continuous_wake_mode | bool | 是否启动持续唤醒模型 | 否       | true/false | false |
| asr_pub_topic_name   | std::string | ASR结果发布话题    | 否       | 根据实际情况配置 | /prompt_text   |
| tts_sub_topic_name   | std::string | TTS内容接收话题    | 否       | 根据实际情况配置 | /tts_text   |
```

# 常见问题

1. 无法打开音频设备？

- 确认音频设备连接是否正常
- 确认是否正确配置音频设备
