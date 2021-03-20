# 自动步兵通信代码解析
    前言：
    本文档将从通俗角度解释RoboMaster自动步兵的通信机制
    代码位置：roborts_base
    注：可通过目录快速跳转到你感兴趣的位置

---
* 作者：c-intelligent
* 撰写日期：2022/3/15
* 联系方式：1842306143@qq.com
* [GitHub](https://github.com/C-Intelligent/RoboMaster/tree/auto_infantry)

---

<br />
## 通信机制概述

---
首先要说明的问题是，谁和谁通信。这里至少有两分方面: 上位机程序中各个构件间的通信问题；上位机与下位机间的通信

### 上下位机间的通信
#### 硬件条件
- 串口通信，且是全双工方式
#### 上位机数据发送
- 上位机底层使用c++标准 write()函数向端口写入字节流。
- 这里我们自然会发问，你什么时候向端口写字节流呢？
- 答案是ros节点发布相应topic的时候，回调函数中执行**相关程序**立刻给他写到端口里面去。当然这只是主要情况，细节稍候还会谈及。同样的，**相关程序**也并不简单，之后会解释。
- 发送过程总结以下就是节点一发布数据我们就写进串口，它在宏观上是并发的，但其实微观上是交替的
#### 上位机数据接收
- 这里是一直有一个**线程**随时处理接收信道中的字节流的，简单来说就是把串口中的数据提取到缓冲区
- 注意，现在我们得到的只是字节流，没有解包是无法使用的。在得到字节流后解包程序会将其**解包**并挂到**map**上
- 最后要说数据包的消费，上层的死循环会不断从map中取出数据包并通过**ros通信机制**再次广播出去

### 上位机构件间的通信
- 上位机程序中的的各个构件的通信是通过ros提供的**话题**机制和**广播**机制进行通信
- 其实这里的上下位机间通信也是采用了类似的机制：所有构件共享公共消息管道，消息发送方不指明接收者，无差别发送。接收方自主决定要获取的消息(**订阅**)
- 区别在于上位机获取下位机数据包的的对应关系是一对一的，且获取的时刻就是数据包消亡的时刻

<br />
## 相关代码说明
---

### 数据发送
#### Write
- /src/roborts_base/roborts_sdk/hardware/serial_device.cpp
```cpp
int SerialDevice::Write(const uint8_t *buf, int len) {
  return write(serial_fd_, buf, len);
}
```
- 这是最靠近硬件层的上位机代码，最终执行写入串口的函数

#### DeviceSend
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp
```cpp
bool Protocol::DeviceSend(uint8_t *buf) {
  int ans;
  Header *header_ptr = (Header *) buf;

  ans = serial_device_ptr_->Write(buf, header_ptr->length);

  if (ans <= 0) {
    DLOG_ERROR << "Port failed.";
  } else if (ans != header_ptr->length) {
    DLOG_ERROR << "Port send failed, send length:" << ans << "package length" << header_ptr->length;
  } else {
    DLOG_INFO << "Port send success with length: " << header_ptr->length;
    return true;
  }
  return false;
}
```
- Write的上层函数

#### SendCMD
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp

```cpp
bool Protocol::SendCMD(uint8_t cmd_set, uint8_t cmd_id, uint8_t receiver,
                       void *data_ptr, uint16_t data_length,
                       CMDSessionMode session_mode, MessageHeader* message_header,
                       std::chrono::milliseconds ack_timeout, int retry_time) {

  CMDSession *cmd_session_ptr = nullptr;
  Header *header_ptr = nullptr;
  uint8_t cmd_set_prefix[] = {cmd_id, cmd_set};
  uint32_t crc_data;

  uint16_t pack_length = 0;


  //calculate pack_length first
  if (data_length == 0 || data_ptr == nullptr) {
    DLOG_ERROR << "No data send.";
    return false;
  }
  pack_length = HEADER_LEN +
      CMD_SET_PREFIX_LEN +
      data_length + CRC_DATA_LEN;

  ROS_INFO("pack length: %d head_len : %d prefix %d data %d crc %d", 
    pack_length, HEADER_LEN, CMD_SET_PREFIX_LEN, data_length, CRC_DATA_LEN);
  //second get the param into the session
  switch (session_mode) {
    //不需要ACK
    case CMDSessionMode::CMD_SESSION_0:
      //lock
      memory_pool_ptr_->LockMemory();
      cmd_session_ptr = AllocCMDSession(CMDSessionMode::CMD_SESSION_0, pack_length);

      if (cmd_session_ptr == nullptr) {
        //unlock
        memory_pool_ptr_->UnlockMemory();
        DLOG_ERROR << "Allocate CMD session failed.";
        return false;
      }

      //pack into cmd_session memory_block
      //头部 12个字节
      header_ptr = (Header *) cmd_session_ptr->memory_block_ptr->memory_ptr;
      header_ptr->sof = SOF;
      header_ptr->length = pack_length;
      header_ptr->version = VERSION;
      header_ptr->session_id = cmd_session_ptr->session_id;
      header_ptr->is_ack = 0;
      header_ptr->reserved0 = 0;
      header_ptr->sender = DEVICE;
      header_ptr->receiver = receiver;
      header_ptr->reserved1 = 0;
      header_ptr->seq_num = seq_num_;
      header_ptr->crc = CRC16Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, HEADER_LEN - CRC_HEAD_LEN);

      if(message_header){
        message_header->is_ack = false;
        message_header->seq_num = seq_num_;
        message_header->session_id = cmd_session_ptr->session_id;
      }

      // pack the cmd prefix ,data and data crc into memory block one by one
      //prefix : 2字节
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN, cmd_set_prefix, CMD_SET_PREFIX_LEN);
      //data : 
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + HEADER_LEN + CMD_SET_PREFIX_LEN, data_ptr, data_length);
      //crc : 4字节
      crc_data = CRC32Calc(cmd_session_ptr->memory_block_ptr->memory_ptr, pack_length - CRC_DATA_LEN);
      memcpy(cmd_session_ptr->memory_block_ptr->memory_ptr + pack_length - CRC_DATA_LEN, &crc_data, CRC_DATA_LEN);

      // send it using device
      DeviceSend(cmd_session_ptr->memory_block_ptr->memory_ptr);

      seq_num_++;
      FreeCMDSession(cmd_session_ptr);
      //unlock
      memory_pool_ptr_->UnlockMemory();
      break;

    default:DLOG_ERROR << "session mode is not valid";
      return false;
  }

  return true;

}
```
- DeviceSend的上层函数
- 注意：这里只截取了部分（case CMDSessionMode::CMD_SESSION_0），其余部分目前并不会用到
- 这是最重要的打包函数，上位机消息在这里被打包成了数据包
- 数据包分为四个部分: head, cmd_prefix, data, crc_data
- 同一层次的发送函数还有SendACK，暂时不展开叙述

#### SendMessage
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp

```cpp
bool Protocol::SendMessage(const CommandInfo *command_info,
                           void *message_data) {
  return SendCMD(command_info->cmd_set, command_info->cmd_id,
                 command_info->receiver, message_data, command_info->length,
                 CMDSessionMode::CMD_SESSION_0);
}
```
- SendCMD的上层函数

#### Publish
- /src/roborts_base/roborts_sdk/dispatch/dispatch.h

```cpp
  void Publish(Cmd &message) {
    bool ret = GetHandle()->GetProtocol()->SendMessage(GetCommandInfo().get(), &message);
    if (!ret) {
      DLOG_ERROR << "send message failed!";
    }
  }
```
- SendMessage的上层函数
- 注意区分**publish**，这是自定义的发布函数
- 调用位置：底盘和云台的相应回调函数中
```cpp
例:
void Gimbal::GimbalAngleCtrlCallback(const roborts_msgs::GimbalAngle::ConstPtr &msg){

  roborts_sdk::cmd_gimbal_angle gimbal_angle;
  gimbal_angle.ctrl.bit.pitch_mode = msg->pitch_mode;
  gimbal_angle.ctrl.bit.yaw_mode = msg->yaw_mode;
  gimbal_angle.pitch = msg->pitch_angle*1800/M_PI;
  gimbal_angle.yaw = msg->yaw_angle*1800/M_PI;

  gimbal_angle_pub_->Publish(gimbal_angle);

}
```
- 这样的调用方式决定了它宏观并发的特性，但是由于信道只有一个，它在微观上必然是交替的

#### AutoRepeatSendCheck
- DeviceSend的上层函数
- Check whether the sent command with need for ack gets its ack back and automatic retry sending command
- 由于**ack**机制会大大增加代码难度，故初期暂不考虑启用
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp

```cpp
void Protocol::AutoRepeatSendCheck() {
  while (running_) {
    unsigned int i;

    std::chrono::steady_clock::time_point current_time_stamp;

    for (i = 1; i < SESSION_TABLE_NUM; i++) {

      //LOG_ERROR << "sleep\r\n";
      //sleep(3);
      if (cmd_session_table_[i].usage_flag == 1) {
        current_time_stamp = std::chrono::steady_clock::now();
        if ((std::chrono::duration_cast<std::chrono::milliseconds>
            (current_time_stamp - cmd_session_table_[i].pre_time_stamp) >
            cmd_session_table_[i].ack_timeout)) {

          memory_pool_ptr_->LockMemory();
          if (cmd_session_table_[i].retry_time > 0) {

            if (cmd_session_table_[i].sent >= cmd_session_table_[i].retry_time) {
              LOG_ERROR << "Sending timeout, Free session "
                        << static_cast<int>(cmd_session_table_[i].session_id);
              FreeCMDSession(&cmd_session_table_[i]);
            } else {
              //LOG_ERROR << "Retry session "
              //          << static_cast<int>(cmd_session_table_[i].session_id);
              //DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
              cmd_session_table_[i].pre_time_stamp = current_time_stamp;
              cmd_session_table_[i].sent++;
            }
          } else {
            DLOG_ERROR << "Send once " << i;
            DeviceSend(cmd_session_table_[i].memory_block_ptr->memory_ptr);
            cmd_session_table_[i].pre_time_stamp = current_time_stamp;
          }
          memory_pool_ptr_->UnlockMemory();
        } else {
//        DLOG_INFO<<"Wait for timeout Session: "<< i;
        }
      }
    }
    usleep(1000);
  }
}
```
<br />

### 数据接收
#### Read
- /src/roborts_base/roborts_sdk/hardware/serial_device.cpp

```cpp
int SerialDevice::Read(uint8_t *buf, int len) {
  int ret = -1;

  if (NULL == buf) {
    return -1;
  } else {
    ret = read(serial_fd_, buf, len);
    DLOG_INFO<<"Read once length: "<<ret;
    while (ret == 0) {
      LOG_ERROR << "Connection closed, try to reconnect.";
      while (!Init()) {
        usleep(500000);
      }
      LOG_INFO << "Reconnect Success.";
      ret = read(serial_fd_, buf, len);
    }
    return ret;
  }
}
```
- 底层接收函数，直接读取端口信息

#### Receive
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp
```cpp
RecvContainer *Protocol::Receive() {

  //! Bool to check if the protocol parser has finished a full frame
  bool is_frame = false;

  //! Step 1: Check if the buffer has been consumed
  if (recv_buff_read_pos_ >= recv_buff_read_len_) {
    recv_buff_read_pos_ = 0;
    recv_buff_read_len_ = serial_device_ptr_->Read(recv_buff_ptr_, BUFFER_SIZE);
  }

  //! Step 2:
  //! For large data protocol, store the value and only verify the header
  //! For small data protocol, Go through the buffer and return when you
  //! see a full frame. buf_read_pos will maintain state about how much
  //! buffer data we have already read

  //TODO: unhandled
  if (is_large_data_protocol_ && recv_buff_read_len_ == BUFFER_SIZE) {

    memcpy(recv_stream_ptr_->recv_buff + (recv_stream_ptr_->recv_index), recv_buff_ptr_,
           BUFFER_SIZE);
    recv_stream_ptr_->recv_index += BUFFER_SIZE;
    recv_buff_read_pos_ = BUFFER_SIZE;
  } else {
    for (recv_buff_read_pos_; recv_buff_read_pos_ < recv_buff_read_len_;
         recv_buff_read_pos_++) {
      is_frame = ByteHandler(recv_buff_ptr_[recv_buff_read_pos_]);

      if (is_frame) {
        DLOG_ERROR << "recieve frame succeed!\r\n";
        return recv_container_ptr_;
      }
    }
  }

  ROS_INFO("receive fail!");
  //! Step 3: If we don't find a full frame by this time, return nullptr.
  return nullptr;
}
```
- Receive函数从端口将字节流读进buffer,并且判断是否含有完整数据帧
- 直到存在完整数据帧，将其解析并返回指向此数据帧的指针

#### ReceivePool
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp
```cpp
void Protocol::ReceivePool() {
  std::chrono::steady_clock::time_point start_time, end_time;
  std::chrono::microseconds execution_duration;
  std::chrono::microseconds cycle_duration = std::chrono::microseconds(int(1e6/READING_RATE));
  while (running_) {
    start_time = std::chrono::steady_clock::now();
    RecvContainer *container_ptr = Receive();
    if (container_ptr) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (buffer_pool_map_.count(std::make_pair(container_ptr->command_info.cmd_set,
                                                container_ptr->command_info.cmd_id)) == 0) {
        buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                        container_ptr->command_info.cmd_id)]
            = std::make_shared<CircularBuffer<RecvContainer>>(100);

        DLOG_INFO<<"Capture command: "
                 <<"cmd set: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_set)
                 <<", cmd id: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.cmd_id)
                 <<", sender: 0x"<< std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.sender)
                 <<", receiver: 0x" <<std::setw(2) << std::hex << std::setfill('0') << int(container_ptr->command_info.receiver);

      }
      //1 time copy
      buffer_pool_map_[std::make_pair(container_ptr->command_info.cmd_set,
                                      container_ptr->command_info.cmd_id)]->Push(*container_ptr);
    }
    end_time = std::chrono::steady_clock::now();
    std::chrono::microseconds execution_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
    if (cycle_duration > execution_duration){
      std::this_thread::sleep_for(cycle_duration - execution_duration);
    }

  }
}
```
- Receive上层函数
- ReceivePool在单独线程中运行，接收数据，并挂到**map**上

```cpp
  receive_pool_thread_ = std::thread(&Protocol::ReceivePool, this);
```

#### Take
- 现在我们已经知道了通信数据怎样达到上位机，并且被存到map中，那它又是怎样被取走的呢，就是Take函数
- /src/roborts_base/roborts_sdk/protocol/protocol.cpp
```cpp
bool Protocol::Take(const CommandInfo *command_info,
                    MessageHeader *message_header,
                    void *message_data) {

  std::lock_guard<std::mutex> lock(mutex_);
  if (buffer_pool_map_.count(std::make_pair(command_info->cmd_set,
                                            command_info->cmd_id)) == 0) {
//    DLOG_ERROR<<"take failed";
    return false;
  } else {
    //1 time copy
    RecvContainer container;

    if (!buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                         command_info->cmd_id)]->Pop(container)) {
//      DLOG_EVERY_N(ERROR, 100)<<"nothing to take";
      return false;
    }


    bool mismatch = false;

    if (int(container.command_info.need_ack) != int(command_info->need_ack)){
      DLOG_ERROR << "Requested need_ack: "<< int(command_info->need_ack)
                 << ", Get need_ack: "<< int(container.command_info.need_ack);
      mismatch = true;
    }

    if (container.message_header.is_ack){
      if (int(container.command_info.receiver) != int(command_info->sender)){
        DLOG_ERROR << "Requested ACK receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->sender)
                   << ", Get ACK receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver);
        mismatch = true;
      }
      if (int(container.command_info.sender) != int(command_info->receiver)){
        DLOG_ERROR << "Requested ACK sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->receiver)
                   << ", Get ACK sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.sender);
        mismatch = true;
      }
    }
    else{
      if (int(container.command_info.receiver) != int(command_info->receiver)){
        DLOG_ERROR << "Requested receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver)
                   << ", Get receiver: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.receiver);
        mismatch = true;
      }

      if (int(container.command_info.sender) != int(command_info->sender)){
        DLOG_ERROR << "Requested sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(command_info->sender)
                   << ", Get sender: "<< std::setw(2) << std::hex << std::setfill('0') << int(container.command_info.sender);
        mismatch = true;
      }
    }

    if (int(container.command_info.length) !=int(command_info->length)){
      DLOG_ERROR << "Requested length: "<< int(command_info->length)
                 <<", Get length: "<< int(container.command_info.length);
      mismatch = true;
    }

    if(mismatch){
      buffer_pool_map_[std::make_pair(command_info->cmd_set,
                                      command_info->cmd_id)]->Push(container);
      return false;
    }

    //1 time copy
    memcpy(message_header, &(container.message_header), sizeof(message_header));
    memcpy(message_data, &(container.message_data), command_info->length);

    return true;
  }
}
```
- 这里请忽略它的底层实现，你只需要知道通过它可以将我们之前挂到map上的数据对应的**摘走**

#### Execue
- 看完上一条，我们一定好奇Take又是在哪里被调用的，就是标题函数
- 注意：这不是一个函数，而是三个
- /src/roborts_base/roborts_sdk/dispatch/execution.cpp

```cpp
void Executor::ExecuteSubscription(const std::shared_ptr<SubscriptionBase>& subscription) {
  auto message_header = subscription->CreateMessageHeader();
  std::shared_ptr<void> message = subscription->CreateMessage();

  bool ret = GetHandle()->GetProtocol()->Take(
      subscription->GetCommandInfo().get(),
      message_header.get(),
      message.get());
  if (ret) {
    subscription->HandleMessage(message_header, message);
  } else {
//      DLOG_ERROR<<"take message failed!";
  }
  //TODO: add return message;
  //subscription->return_message(message);
}
void Executor::ExecuteService(const std::shared_ptr<ServiceBase>& service) {
  auto request_header = service->CreateRequestHeader();
  std::shared_ptr<void> request = service->CreateRequest();

  bool ret = GetHandle()->GetProtocol()->Take(
      service->GetCommandInfo().get(),
      request_header.get(),
      request.get());
  if (ret) {
    service->HandleRequest(request_header, request);
  } else {
    DLOG_ERROR << "take request failed!";
  }
}
void Executor::ExecuteClient(const std::shared_ptr<ClientBase>& client) {
  auto request_header = client->CreateRequestHeader();
  std::shared_ptr<void> response = client->CreateResponse();

  bool ret = GetHandle()->GetProtocol()->Take(
      client->GetCommandInfo().get(),
      request_header.get(),
      response.get());

  if (ret) {
    client->HandleResponse(request_header, response);
  } else {
//      DLOG_ERROR<<"take response failed!";
  }
}
```
- 这里姑且称呼为执行器。每一次调用，执行器会取出自己所需要的相应数据包，此时数据包也算是到了消亡的时候。执行器的调用写在ros节点的Spin()函数中

#### Spin
- Spin函数是我们的老朋友了，通俗来说就是节点维护的死循环
- /src/roborts_base/roborts_sdk/dispatch/handle.cpp
```cpp
void Handle::Spin() {

  for (auto sub :subscription_factory_) {
    executor_->ExecuteSubscription(sub);
  }
  for (auto client :client_factory_) {
    executor_->ExecuteClient(client);
  }
  for (auto service :service_factory_) {
    executor_->ExecuteService(service);
  }
}
```
- 就是在这个顶层循环中，消息源源不断被取走并且发送到ros话题中被各个构件使用，但是你估计还有疑问：这都到顶层了，它是在哪里被publish的呢，或许细心一点已经注意到了之前execute中还有一个handle函数，下一条会解释
- 而且，你也可能会对这里的for循环产生疑惑，这是啥玩意儿？subscription_factory_
- 这里先简单解释一下，比如第一个，是一个订阅者工厂，每次成批地处理订阅者的订阅回调函数
- 因此ExecuteSubscription显然是一个模板函数

#### HandleMessage & Subscription & SubscriptionCallback
- 其实同层次的函数还有**HandleRequest**, **HandleResponse**
- 但是我们嘴关注的还是HandleMessage，而且另外两个暂时可能不会使用
- /src/roborts_base/roborts_sdk/dispatch/dispatch.h

```cpp
template<typename Cmd>
class Subscription : public SubscriptionBase {
 public:
  using SharedMessage = typename std::shared_ptr<Cmd>;
  using CallbackType = typename std::function<void(const SharedMessage)>;

  Subscription(std::shared_ptr<Handle> handle,
               uint8_t cmd_set, uint8_t cmd_id,
               uint8_t sender, uint8_t receiver,
               CallbackType &&function) :
      SubscriptionBase(handle, cmd_set, cmd_id, sender, receiver),
      callback_(std::forward<CallbackType>(function)) {
    cmd_info_->length = sizeof(Cmd);
  }
  ~Subscription() = default;
  std::shared_ptr<void> CreateMessage() {
    return std::shared_ptr<void>(new Cmd);
  }
  void HandleMessage(std::shared_ptr<MessageHeader> message_header, std::shared_ptr<void> message) {
    auto typed_message = std::static_pointer_cast<Cmd>(message);
    callback_.Dispatch(message_header, typed_message);
  }
 private:
  SubscriptionCallback<Cmd> callback_;
};
```
- 这里贴出的是整个订阅者类，HandleMessage是Subscription的成员函数
- 上一条已经谈到了，我们会使用HandleMessage来处理数据包
- 我们不妨先看看Dispatch是何方神圣
```cpp
template<typename Cmd>
class SubscriptionCallback {
 public:
  using SharedMessage = typename std::shared_ptr<Cmd>;
  using CallbackType = typename std::function<void(const SharedMessage)>;

  SubscriptionCallback(CallbackType function) :
      callback_function_(function) {

  }
  ~SubscriptionCallback() = default;
  void Dispatch(std::shared_ptr<MessageHeader> message_header,
                SharedMessage message) {
    callback_function_(message);
  };
 private:
  CallbackType callback_function_;
};
```
- 玩牍子，里面也没有调用ros的publish啊!!
- 注意看字面意思，callback_function_(message)，将消息传入回调函数，而这里的callback_function_是一个函数成员（注：std::function，通用多态函数包装器，其实例可以存储复制任何函数），相当于在这里调用了这个函数。
- 于是我们又会发问，哪赋值的呢？
  - 答：对象实例化的时候。
- 所以它到底是什么函数呢？（请看下下一条）。
- SubscriptionCallback哪里被实例化的呢？
  - 答：Subscription实例化的时候， 因为SubscriptionCallback是Subscription的成员
- Subscription在哪里实例化呢？(请看下一条)

#### CreateSubscriber
- /src/roborts_base/roborts_sdk/dispatch/handle.h
```cpp
  template<typename Cmd>
  std::shared_ptr<Subscription<Cmd>> CreateSubscriber(uint8_t cmd_set, uint8_t cmd_id,
                                                      uint8_t sender, uint8_t receiver,
                                                      typename Subscription<Cmd>::CallbackType &&function) {
                                                        //shared_from_this() 返回一个当前类的std::share_ptr,
    auto subscriber = std::make_shared<Subscription<Cmd>>(shared_from_this(),
                                                          cmd_set, cmd_id,
                                                          sender, receiver,
                                                          std::forward<typename Subscription<Cmd>::CallbackType>(
                                                              function));
    subscription_factory_.push_back(
        std::dynamic_pointer_cast<SubscriptionBase>(subscriber));
    return subscriber;
  }
```
- 那么我们在这里知道了subscriber是通过CreateSubscriber创建的，并且会放到subscription_factory_中，到这里，我们之前遇到的for循环中调取工厂成员的疑惑可以解除了
- 那么还剩最后一个问题，它在哪里被调用，又是传进的怎样的回调函数
- 注意：请区分此Subscriber和ros中的subscriber，以及此回调函数，与ros中的回调函数是有区别的

#### SDK_Init
- /src/roborts_base/chassis/chassis.cpp
- /src/roborts_base/gimbal/gimbal.cpp
- /src/roborts_base/referee_system/referee_system.cpp
- 这里的三个SDK_Init具有相似性，只谈一个
```cpp
void Gimbal::SDK_Init(){

  verison_client_ = handle_->CreateClient<roborts_sdk::cmd_version_id,roborts_sdk::cmd_version_id>
      (UNIVERSAL_CMD_SET, CMD_REPORT_VERSION,
       MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  roborts_sdk::cmd_version_id version_cmd;
  version_cmd.version_id=0;
  auto version = std::make_shared<roborts_sdk::cmd_version_id>(version_cmd);
  ROS_INFO("AsyncSendRequest");
  verison_client_->AsyncSendRequest(version,
                                    [](roborts_sdk::Client<roborts_sdk::cmd_version_id,
                                                           roborts_sdk::cmd_version_id>::SharedFuture future) {
                                      ROS_INFO("Gimbal Firmware Version: %d.%d.%d.%d",
                                               int(future.get()->version_id>>24&0xFF),
                                               int(future.get()->version_id>>16&0xFF),
                                               int(future.get()->version_id>>8&0xFF),
                                               int(future.get()->version_id&0xFF));
                                    });

  handle_->CreateSubscriber<roborts_sdk::cmd_gimbal_info>(GIMBAL_CMD_SET, CMD_PUSH_GIMBAL_INFO,
                                                          GIMBAL_ADDRESS, BROADCAST_ADDRESS,
                                                          std::bind(&Gimbal::GimbalInfoCallback, this, std::placeholders::_1));

  gimbal_angle_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_gimbal_angle>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_ANGLE,
                                                                              MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_mode_pub_ = handle_->CreatePublisher<roborts_sdk::gimbal_mode_e>(GIMBAL_CMD_SET, CMD_SET_GIMBAL_MODE,
                                                                          MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  fric_wheel_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_fric_wheel_speed>(GIMBAL_CMD_SET, CMD_SET_FRIC_WHEEL_SPEED,
                                                                                MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  gimbal_shoot_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_shoot_info>(GIMBAL_CMD_SET, CMD_SET_SHOOT_INFO,
                                                                            MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);

  heartbeat_pub_ = handle_->CreatePublisher<roborts_sdk::cmd_heartbeat>(UNIVERSAL_CMD_SET, CMD_HEARTBEAT,
                                                                        MANIFOLD2_ADDRESS, GIMBAL_ADDRESS);
  heartbeat_thread_ = std::thread([this]{
                                    roborts_sdk::cmd_heartbeat heartbeat;
                                    heartbeat.heartbeat=0;
                                    while(ros::ok()){
                                      //heartbeat_pub_->Publish(heartbeat);
                                      std::this_thread::sleep_for(std::chrono::milliseconds(300));
                                    }
                                  }
  );
}
```
- subscriber就是在这里被实例化的，并且传入了回调函数Gimbal::GimbalInfoCallback

```cpp
// --/src/roborts_base/gimbal/gimbal.cpp
void Gimbal::GimbalInfoCallback(const std::shared_ptr<roborts_sdk::cmd_gimbal_info> gimbal_info){

  ros::Time current_time = ros::Time::now();
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromRollPitchYaw(0.0,
                                                                        gimbal_info->pitch_ecd_angle / 1800.0 * M_PI,
                                                                        gimbal_info->yaw_ecd_angle / 1800.0 * M_PI);
  gimbal_tf_.header.stamp = current_time;
  gimbal_tf_.transform.rotation = q;
  gimbal_tf_.transform.translation.x = 0;
  gimbal_tf_.transform.translation.y = 0;
  gimbal_tf_.transform.translation.z = 0.15;
  tf_broadcaster_.sendTransform(gimbal_tf_);

}
```
- 可以看到，回调函数获取消息后通过ros消息机制进行了广播，ros构件的通信也建立了
- 至此，整个通信过程应该已经串起来了

<br />

<!-->
### 初始化
- 这部分不是我们的探讨重点，就简要介绍一下调用关系
</-->

<br />

## 协议说明
---
- 一帧协议数据分为 4 个部分，分别是帧头数据、命令码ID、数据、帧尾校验数据。

### 帧头数据  

``` c
/** 
  * @brief  frame header structure definition
  */
/* This Struct Is Used To Describe A Package Header */
typedef struct
{
  uint8_t sof; /*!< Identify Of A Package */
  union {
    struct
    {
      uint16_t data_len : 10; /*!< Data Length, Include Header And Crc */
      uint16_t version : 6;   /*!< Protocol Version */
    };
    uint16_t ver_data_len;
  };
  union {
    struct
    {
      uint8_t session : 5;   /*!< Need(0~1) Or Not Need(2~63) Ack */
      uint8_t pack_type : 1; /*!< Ack Package Or Normal Package */
      uint8_t res : 2;       /*!< Reserve */
    };
    uint8_t S_A_R_c;
  };
  uint8_t sender;   /*!< Sender Module Information */
  uint8_t reciver;  /*!< Receiver Module Information */
  uint16_t res1;    /*!< Reserve 1 */
  uint16_t seq_num; /*!< Sequence Number */
  uint16_t crc_16;  /*!< CRC16 */
  uint8_t pdata[];
} protocol_pack_desc_t;
```


| 帧头数据      | 占用字节 | 详细描述                            |
| :------------| :-------| :--------------------------------- |
| sof          | 1       | 数据的域ID                          |
| ver_data_len | 2       | 每帧内数据的长度和协议版本号          |
| session      | 1       | 包序号，在0xA0域中保留               |
| sender       | 1       |发送者地址                          |
| reciver      | 1       |发送者地址                          |
| res          | 2       |保留位                              |
| seq          | 2       |包序号                              |
| crc16        | 2       | 帧头的crc校验结果                   |


### 命令码

| 命令码   | 占用字节 |
| :---- | :--- |
| cmdid | 2    |

- 一个命令码对应一帧包含具体信息的数据，下面是现有所有数据对应的命令码。


- 命令码对应的数据传输方向和具体功能如下：

| 命令码    | 传输方向    | 功能介绍             | 频率             |
| :----- | :------ | :--------------- | :------------- |
| 0x0001 | 主控-->PC | 比赛时机器人状态         | 裁判系统10Hz       |
| 0x0002 | 主控-->PC | 实时伤害数据           | 受到攻击时发送        |
| 0x0003 | 主控-->PC | 实时射击数据           | 裁判系统           |
| 0x0004 | 主控-->PC | 实时功率、热量数据        | ICRA不使用，不发送    |
| 0x0005 | 主控-->PC | 场地 RFID 数据       | 检测到 IC 卡发送     |
| 0x0006 | 主控-->PC | 比赛结果数据           | 比赛结束时发送        |
| 0x0007 | 主控-->PC | 获得 buff 数据       | 裁判系统           |
| 0x0008 | 主控-->PC | 场地 UWB 数据        | 裁判系统           |
|        |           |                  |                |
| 0x0204 | 主控-->PC | 机器人底盘相关信息        | 100Hz定频         |
| 0x0304 | 主控-->PC | 机器人云台相关信息        | 100Hz定频         |
| 0x0402 | 主控-->PC | UWB相关信息        | UWB更新频率         |
|        |           |                  |                |
| 0x0206 | PC-->主控 | 设置底盘速度        |          |
| 0x0208 | PC-->主控 | 设置底盘速度(有加速度)  |      |
| 0x0309 | PC-->主控 | 控制摩擦轮转速         | 开启摩擦轮使用         |
| 0x030A | PC-->主控 | 控制射击     |         |
| 0x0403 | PC-->主控 | 云台相关校准信息         | 需要校准云台时发送      |

### 数据  

- 为命令码 ID 对应的数据结构，数据长度即这个结构体的大小。

    | 数据   | 占用字节        |
    | :--- | :---------- |
    | data | data_length |
---
#### 第一类

- 裁判系统学生串口信息，详情查阅裁判系统手册

#### 第二类

- 控制信息与推送信息：
- application/infantry_cmd.h
- /src/roborts_base/roborts_sdk/protocol/protocol_define.h

```c

struct cmd_chassis_info
{
  int16_t gyro_angle;
  int16_t gyro_palstance;
  int32_t position_x_mm;
  int32_t position_y_mm;
  int16_t angle_deg;
  int16_t v_x_mm;
  int16_t v_y_mm;
};

struct cmd_gimbal_info
{
  uint8_t   mode;
  /* unit: degree */
  int16_t pitch_ecd_angle;
  int16_t yaw_ecd_angle;
  int16_t pitch_gyro_angle;
  int16_t yaw_gyro_angle;
  /* uint: degree/s */
  int16_t yaw_rate;
  int16_t pitch_rate;
};

struct cmd_gimbal_angle
{
  union{
    uint8_t flag;
    struct{
        uint8_t yaw_mode:1;  // 0 code angle
        uint8_t pitch_mode:1;
    }bit;
  } ctrl;
  int16_t pitch;
  int16_t yaw;
};

struct cmd_chassis_speed
{
  int16_t vx; // forward/back
  int16_t vy; // left/right
  int16_t vw; // anticlockwise/clockwise
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_chassis_spd_acc
{
  int16_t   vx; 
  int16_t   vy;
  int16_t   vw; 

  int16_t   ax; 
  int16_t   ay; 
  int16_t   wz; 
  
  int16_t rotate_x_offset;
  int16_t rotate_y_offset;
};

struct cmd_firction_speed
{
  uint16_t left;
  uint16_t right;
};

struct cmd_shoot_num
{
  uint8_t  shoot_cmd;
  uint32_t shoot_add_num;
  uint16_t shoot_freq;
};

```
- 如上图所见，这几个数据包就是上下位机间通信的数据包


<br />

## 下位机通信代码
---
### 发送
```c
int32_t protocol_send_cmd_config(uint16_t cmd,
                                 uint8_t resend_times,
                                 uint16_t resend_timeout,
                                 uint8_t ack_enable,
                                 ack_handle_fn_t ack_callback,
                                 no_ack_handle_fn_t no_ack_callback);

int32_t protocol_rcv_cmd_register(uint16_t cmd, rcv_handle_fn_t rcv_callback);
int32_t protocol_rcv_cmd_unregister(uint16_t cmd);
int32_t protocol_send_cmd_unregister(uint16_t cmd);
uint32_t protocol_send_flush(void);
uint32_t protocol_unpack_flush(void);
uint32_t protocol_send_list_add_callback_reg(void_fn_t fn);

int32_t protocol_can_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t broadcast_output_enable,
                                        uint8_t can_port,
                                        uint32_t can_tx_id,
                                        uint32_t can_rx_id,
                                        int (*can_send_fn)(uint32_t std_id, uint8_t *p_data, uint32_t len));
int32_t protocol_uart_interface_register(char *interface_name,
                                        uint16_t rcv_buf_size,
                                        uint8_t broadcast_output_enable,
                                        uint8_t com_port,
                                        int (*com_send_fn)(uint8_t *p_data, uint32_t len));

int32_t protocol_set_route(uint8_t tar_add, const char *name);
uint32_t protocol_can_rcv_data(uint8_t can_port, uint32_t rcv_id, void *p_data, uint32_t data_len);
uint32_t protocol_uart_rcv_data(uint8_t com_port, void *p_data, uint32_t data_len);

```

<br />
### 接收
```c

void read_and_unpack_thread(void *argu)
{
  uint8_t byte = 0;
  int32_t read_len;
  int32_t buff_read_index;
  
  uint16_t      data_len;
  unpack_step_e unpack_step;
  int32_t       index;
  uint8_t       protocol_packet[PROTOCAL_FRAME_MAX_SIZE];

  while (1)
  {
    read_len = uart_recv(uart_fd, computer_rx_buf, UART_BUFF_SIZE);
    buff_read_index = 0;
    
    while (read_len--)
    {
      byte = computer_rx_buf[buff_read_index++];
      
      switch(unpack_step)
      {
        case STEP_HEADER_SOF:
        {
          if(byte == UP_REG_ID)
          {
            unpack_step = STEP_LENGTH_LOW;
            protocol_packet[index++] = byte;
          }
          else
          {
            index = 0;
          }
        }break;
        
        case STEP_LENGTH_LOW:
        {
          data_len = byte;
          protocol_packet[index++] = byte;
          unpack_step = STEP_LENGTH_HIGH;
        }break;
        
        case STEP_LENGTH_HIGH:
        {
          data_len |= (byte << 8);
          protocol_packet[index++] = byte;

          if(data_len < (PROTOCAL_FRAME_MAX_SIZE - HEADER_LEN - CRC_LEN))
          {
            unpack_step = STEP_FRAME_SEQ;
          }
          else
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;
          }
        }break;
      
        case STEP_FRAME_SEQ:
        {
          protocol_packet[index++] = byte;
          unpack_step = STEP_HEADER_CRC8;
        }break;

        case STEP_HEADER_CRC8:
        {
          protocol_packet[index++] = byte;

          if (index == HEADER_LEN)
          {
            if ( verify_crc8_check_sum(protocol_packet, HEADER_LEN) )
            {
              unpack_step = STEP_DATA_CRC16;
            }
            else
            {
              unpack_step = STEP_HEADER_SOF;
              index = 0;
            }
          }
        }break;  

        case STEP_DATA_CRC16:
        {
          if (index < (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
             protocol_packet[index++] = byte;  
          }
          if (index >= (HEADER_LEN + CMD_LEN + data_len + CRC_LEN))
          {
            unpack_step = STEP_HEADER_SOF;
            index = 0;

            if ( verify_crc16_check_sum(protocol_packet, HEADER_LEN + CMD_LEN + data_len + CRC_LEN) )
            {
              data_handle(protocol_packet);
            }
          }
        }break;

        default:
        {
          unpack_step = STEP_HEADER_SOF;
          index = 0;
        }break;
      }
    }
  }
}

void data_handle(uint8_t *p_frame)
{
  frame_header_t *p_header = (frame_header_t*)p_frame;
  memcpy(p_header, p_frame, HEADER_LEN);

  uint16_t data_length = p_header->data_length;
  uint16_t cmd_id      = *(uint16_t *)(p_frame + HEADER_LEN);
  uint8_t *data_addr   = p_frame + HEADER_LEN + CMD_LEN;
  
  switch (cmd_id)
  {
    case GAME_INFO_ID:
      memcpy(&game_information, data_addr, data_length);
    break;
    
    //............
    //............

  }
}
```