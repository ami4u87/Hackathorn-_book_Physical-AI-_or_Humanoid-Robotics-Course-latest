import React from 'react';
import ChatbotButton from '@site/src/components/ChatbotButton';

// This component wraps all pages and allows us to add global elements
export default function Root({children}) {
  return (
    <>
      {children}
      <ChatbotButton />
    </>
  );
}
