import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';
import RAGChatbot from '@site/src/components/RAGChatbot/RAGChatbot';

if (ExecutionEnvironment.canUseDOM) {
  // This will render the chatbot component on all pages
  const renderChatbot = () => {
    const root = document.createElement('div');
    root.id = 'rag-chatbot-root';
    document.body.appendChild(root);
    
    // Use ReactDOM to render the chatbot
    const ReactDOM = require('react-dom/client');
    const reactElement = React.createElement(RAGChatbot);
    const reactRoot = ReactDOM.createRoot(root);
    reactRoot.render(reactElement);
  };
  
  // Wait for the page to be fully loaded before rendering
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', renderChatbot);
  } else {
    renderChatbot();
  }
}