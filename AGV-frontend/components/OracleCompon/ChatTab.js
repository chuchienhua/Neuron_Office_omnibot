import React, { useState } from 'react';
import axios from 'axios';
import './ChatTab.css';

const Carurl = "http://192.168.8.241:3561/";
// const Carurl = "http://192.168.103.128:9090/";

const ChatTab = () => {
 const [userMessage, setUserMessage] = useState('');
 const [messages, setMessages] = useState({}); // 改為物件
 const [carId, setCarId] = useState(0x01);

 const sendMessage = () => {
  axios
    .post(Carurl + "chatgpt_msg", {
      carid: carId,
      chatgpt_msg: userMessage,
    })
    .then((response) => {
      const responseMessage = response.data;
      // console.log(response);
      setMessages(prevMessages => ({
        ...prevMessages,
        [carId]: [...(prevMessages[carId] || []), { type: 'user', text: userMessage }, { type: 'response', text: responseMessage.chatgpt_location_name }]
      }));
      setUserMessage('');
    });
 };

 const clearChat = () => {
    setMessages(prevMessages => ({ ...prevMessages, [carId]: [] }));
    setUserMessage('');
 };

 return (
    <div className="chat-container">
      <div className="input-chatgpt-container">
        <select value={carId} onChange={(e) => setCarId(e.target.value)}>
          <option value="0x01">AGV1</option>
          <option value="0x02">AGV2</option>
          <option value="0x03">AGV3</option>
        </select>
        <input
          type="text"
          value={userMessage}
          onChange={(e) => setUserMessage(e.target.value)}
        />
        <button onClick={sendMessage}>Send</button>
        <button onClick={clearChat}>Clear</button>
      </div>
      <div className="message-container">
        {messages[carId]?.map((message, index) => (
          <div
            key={index}
            className={message.type === 'user' ? 'user-message' : 'chat-gpt-response'}
          >
            {message.text}
          </div>
        ))}
      </div>
    </div>
 );
};

export default ChatTab;
