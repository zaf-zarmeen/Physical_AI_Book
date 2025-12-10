// Client module to add the RAG chatbot to all pages
// This will be executed on every page load

// Create container and render chatbot when DOM is ready
function initializeChatbot() {
  if (typeof document !== 'undefined' && typeof window !== 'undefined') {
    // Check if container already exists to prevent duplicates
    if (document.getElementById('rag-chatbot-container')) {
      return;
    }

    const container = document.createElement('div');
    container.id = 'rag-chatbot-container';
    document.body.appendChild(container);

    // Dynamically import React and ReactDOM
    import('react').then(ReactModule => {
      import('react-dom/client').then(ReactDOMModule => {
        import('./RAGChatbot').then(ChatbotModule => {
          const root = ReactDOMModule.createRoot(container);
          root.render(ReactModule.createElement(ChatbotModule.default));
        });
      });
    });
  }
}

// Initialize when the document is ready
if (typeof document !== 'undefined') {
  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initializeChatbot);
  } else {
    // If already loaded, initialize immediately
    initializeChatbot();
  }
}

// Export a component that does nothing but is needed for Docusaurus to load this module
export default function ChatbotInjector() {
  return null;
}