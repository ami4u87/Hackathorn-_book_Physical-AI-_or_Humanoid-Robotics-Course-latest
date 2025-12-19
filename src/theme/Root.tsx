import React from 'react';
import ChatbotWidget from '../components/ChatbotWidget';

// This component wraps the entire Docusaurus app
// It's the perfect place to add global components like the chatbot
export default function Root({ children }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
