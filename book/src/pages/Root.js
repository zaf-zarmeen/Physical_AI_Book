import React from 'react';
import Layout from '@theme/Layout';
import RAGChatbot from '@site/src/components/RAGChatbot/RAGChatbot';

// Main layout wrapper that includes the RAG chatbot
export default function Root({children}) {
  return (
    <>
      <Layout>
        {children}
      </Layout>
      <RAGChatbot />
    </>
  );
}