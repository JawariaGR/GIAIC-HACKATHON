import React from 'react';
import DocItem from '@theme-original/DocItem';
import ChatbotWidget from '@site/src/components/ChatbotWidget';

export default function DocItemWrapper(props) {
  return (
    <>
      <DocItem {...props} />
      <ChatbotWidget />
    </>
  );
}
