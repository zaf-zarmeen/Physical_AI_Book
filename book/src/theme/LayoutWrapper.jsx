import React from 'react';
import RAGChatbot from '@site/src/components/RAGChatbot/RAGChatbot';

// A layout wrapper that adds the RAG chatbot to all pages
const LayoutWrapper = ({ children }) => {
  return (
    <>
      {children}
      <RAGChatbot />
    </>
  );
};

export default LayoutWrapper;