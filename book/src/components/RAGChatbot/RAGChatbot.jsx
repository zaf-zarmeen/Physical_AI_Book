import React, { useState, useRef, useEffect } from 'react';

/**
 * RAGChatbot Component
 * A Retrieval-Augmented Generation chatbot for the Physical AI textbook
 */
const RAGChatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([
    {
      id: 1,
      text: "Hello! I'm your Physical AI and Robotics assistant. I can answer questions related to the Physical AI & Humanoid Robotics textbook. Ask me about ROS 2, Gazebo, Isaac Sim, VLA systems, or other topics from the textbook.",
      sender: 'bot',
      timestamp: new Date()
    }
  ]);
  const [inputText, setInputText] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const messagesEndRef = useRef(null);

  // Scroll to bottom of messages when they change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const toggleChatbot = () => {
    setIsOpen(!isOpen);
  };

  const handleInputChange = (e) => {
    setInputText(e.target.value);
  };

  const handleSendMessage = async () => {
    if (!inputText.trim() || isLoading) return;

    // Add user message to the chat
    const userMessage = {
      id: messages.length + 1,
      text: inputText,
      sender: 'user',
      timestamp: new Date()
    };

    setMessages(prev => [...prev, userMessage]);
    setInputText('');
    setIsLoading(true);

    try {
      // Simulate API call to RAG system
      // In a real implementation, this would call an actual RAG API
      await new Promise(resolve => setTimeout(resolve, 1000)); // Simulate API delay
      
      // Generate a mock response based on the user's question
      const botResponse = generateMockResponse(inputText);
      
      // Determine relevant sources based on the question
      const sources = [];
      const lowerQuestion = inputText.toLowerCase();

      if (lowerQuestion.includes('ros') || lowerQuestion.includes('node') || lowerQuestion.includes('topic') || lowerQuestion.includes('service')) {
        sources.push({ title: "ROS 2 Fundamentals", url: "/docs/ros2/ros2-fundamentals" });
      }
      if (lowerQuestion.includes('gazebo') || lowerQuestion.includes('simulation')) {
        sources.push({ title: "Gazebo Physics", url: "/docs/simulation/gazebo-physics" });
      }
      if (lowerQuestion.includes('isaac') || lowerQuestion.includes('nvidia')) {
        sources.push({ title: "Isaac Sim Basics", url: "/docs/isaac/isaac-sim-basics" });
      }
      if (lowerQuestion.includes('vla') || lowerQuestion.includes('vision-language-action')) {
        sources.push({ title: "VLA Implementation", url: "/docs/vla-ai/vla-implementation" });
      }
      if (lowerQuestion.includes('capstone') || lowerQuestion.includes('project')) {
        sources.push({ title: "Final Capstone", url: "/docs/capstone/final-capstone" });
      }
      if (lowerQuestion.includes('quiz') || lowerQuestion.includes('test')) {
        if (lowerQuestion.includes('module') || lowerQuestion.includes('1')) {
          sources.push({ title: "Module 1 Quiz", url: "/docs/ros2/quiz-ros2-fundamentals" });
        }
        if (lowerQuestion.includes('module') || lowerQuestion.includes('2')) {
          sources.push({ title: "Module 2 Quiz", url: "/docs/simulation/quiz-simulation" });
        }
        if (lowerQuestion.includes('module') || lowerQuestion.includes('3')) {
          sources.push({ title: "Module 3 Quiz", url: "/docs/isaac/quiz-isaac" });
        }
        if (lowerQuestion.includes('module') || lowerQuestion.includes('4')) {
          sources.push({ title: "Module 4 Quiz", url: "/docs/vla-ai/quiz-vla" });
        }
      }

      // If no specific sources identified, add general textbook source
      if (sources.length === 0) {
        sources.push(
          { title: "Physical AI Textbook", url: "/" },
          { title: "Introduction to Physical AI", url: "/docs/intro/intro-to-physical-ai" }
        );
      }

      const responseMessage = {
        id: messages.length + 2,
        text: botResponse,
        sender: 'bot',
        timestamp: new Date(),
        sources: sources
      };

      setMessages(prev => [...prev, responseMessage]);
    } catch (error) {
      console.error('Error getting response:', error);
      
      const errorMessage = {
        id: messages.length + 2,
        text: "Sorry, I encountered an error processing your request. Please try again.",
        sender: 'bot',
        timestamp: new Date()
      };

      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const generateMockResponse = (question) => {
    const lowerQuestion = question.toLowerCase();

    // Check if the question is related to the textbook topics
    const textbookTopics = [
      'physical ai', 'robotics', 'ros', 'robot operating system', 'gazebo', 'simulation',
      'nvidia isaac', 'isaac sim', 'vla', 'vision-language-action', 'robot', 'ai',
      'artificial intelligence', 'humanoid', 'navigation', 'slam', 'mapping', 'localization',
      'perception', 'manipulation', 'control', 'planning', 'sensors', 'actuators', 'unity',
      'computer vision', 'machine learning', 'deep learning', 'neural networks', 'navigation',
      'path planning', 'computer vision', 'lidar', 'camera', 'imu', 'sensor fusion',
      'domain randomization', 'sim-to-real', 'transfer learning', 'reinforcement learning',
      'manipulator', 'mobile robot', 'wheeled robot', 'legged robot', 'quadruped', 'biped',
      'arm', 'gripper', 'end-effector', 'kinematics', 'dynamics', 'trajectory', 'motion planning'
    ];

    // Check if question contains any textbook-related terms
    const isTextbookRelated = textbookTopics.some(topic => lowerQuestion.includes(topic));

    if (!isTextbookRelated) {
      return "I can only answer questions related to the Physical AI & Humanoid Robotics textbook. I cannot answer questions outside the scope of robotics, AI, ROS 2, Gazebo, Isaac Sim, Vision-Language-Action systems, or related topics covered in the textbook.";
    }

    // If the question is textbook-related, provide informative responses
    if (lowerQuestion.includes('hello') || lowerQuestion.includes('hi')) {
      return "Hello! I'm your Physical AI and Robotics assistant. Ask me about ROS 2, Gazebo, Isaac Sim, VLA systems, or other topics from the textbook.";
    } else if (lowerQuestion.includes('physical ai')) {
      return "Physical AI represents a convergence of artificial intelligence, robotics, and embodied cognition. Unlike traditional AI systems that interact primarily with digital data, Physical AI systems perceive, reason, and act in the physical world using sensors, actuators, AI algorithms, and embodied cognition principles.";
    } else if (lowerQuestion.includes('ros') || lowerQuestion.includes('robot operating system')) {
      return "ROS 2 (Robot Operating System 2) is a flexible framework for writing robot software. It provides services like hardware abstraction, device drivers, libraries, visualizers, message-passing, and package management. For detailed information, see the 'ROS 2 Fundamentals' chapter in the textbook.";
    } else if (lowerQuestion.includes('gazebo') || lowerQuestion.includes('simulation')) {
      return "Gazebo is a physics-based simulation environment that provides realistic sensor simulation and dynamics for Physical AI development. It allows you to test and validate your Physical AI algorithms in a safe, controlled environment before deploying to real robots. For more details, check the 'Gazebo Physics' chapter in the textbook.";
    } else if (lowerQuestion.includes('isaac') || lowerQuestion.includes('nvidia')) {
      return "NVIDIA Isaac Sim is a high-fidelity simulation environment built on NVIDIA Omniverse, specifically designed for robotics development and AI training. It provides photorealistic rendering for synthetic data generation and accurate physics simulation. For more information, see the 'Isaac Sim Basics' chapter in the textbook.";
    } else if (lowerQuestion.includes('vla') || lowerQuestion.includes('vision-language-action')) {
      return "Vision-Language-Action (VLA) systems enable robots to understand natural language commands and execute complex tasks in the physical world. They combine vision for understanding visual information, language processing for understanding commands, and action execution for performing physical behaviors. Details can be found in the 'VLA Implementation' chapter of the textbook.";
    } else if (lowerQuestion.includes('module') && (lowerQuestion.includes('1') || lowerQuestion.includes('one'))) {
      return "Module 1 covers ROS 2 Fundamentals, including nodes, topics, services, actions, and basic robot programming concepts. You can find more details in the 'ROS 2 Fundamentals' chapter.";
    } else if (lowerQuestion.includes('module') && (lowerQuestion.includes('2') || lowerQuestion.includes('two'))) {
      return "Module 2 covers Simulation environments including Gazebo and Unity. It discusses physics simulation, sensor simulation, and how to connect simulation environments to ROS 2. See the 'Gazebo Physics' chapter for more information.";
    } else if (lowerQuestion.includes('module') && (lowerQuestion.includes('3') || lowerQuestion.includes('three'))) {
      return "Module 3 covers NVIDIA Isaac, including Isaac Sim for high-fidelity simulation, perception systems, and navigation. Details are available in the 'Isaac Sim Basics' chapter.";
    } else if (lowerQuestion.includes('module') && (lowerQuestion.includes('4') || lowerQuestion.includes('four'))) {
      return "Module 4 covers Vision-Language-Action systems, including integration of Whisper-1 speech recognition and GPT-based planning. See the 'VLA Implementation' chapter for details.";
    } else if (lowerQuestion.includes('capstone')) {
      return "The capstone project integrates all concepts from the textbook to create a complete Physical AI system. It demonstrates how to combine ROS 2, simulation, Isaac, and VLA systems. See the 'Final Capstone' chapter for the complete project description.";
    } else if (lowerQuestion.includes('quiz') || lowerQuestion.includes('test')) {
      return "Each module in the textbook has a corresponding quiz to test your understanding. You can find quizzes for Module 1 (ROS 2), Module 2 (Simulation), Module 3 (Isaac), Module 4 (VLA), and the Capstone project.";
    } else if (lowerQuestion.includes('personalization') || lowerQuestion.includes('customize')) {
      return "The textbook includes a personalization button that allows you to customize your learning experience with options for theme, font size, language preference, and learning pace.";
    } else if (lowerQuestion.includes('translation') || lowerQuestion.includes('urdu')) {
      return "The textbook includes a translation feature with support for Urdu and English. This is accessible through the translation button in the interface.";
    } else {
      // Default response for textbook-related but not specifically matched questions
      return "I'm here to help with questions related to Physical AI and Robotics. The textbook covers ROS 2, simulation environments like Gazebo and Isaac Sim, Vision-Language-Action systems, and capstone projects. What specific topic would you like to explore?";
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  return (
    <div className="rag-chatbot">
      {/* Chatbot toggle button */}
      <button
        className={`chatbot-toggle ${isOpen ? 'open' : ''}`}
        onClick={toggleChatbot}
        aria-expanded={isOpen}
        aria-label={isOpen ? "Close chatbot" : "Open chatbot"}
      >
        {isOpen ? (
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="12" cy="12" r="10"></circle>
            <line x1="15" y1="9" x2="9" y2="15"></line>
            <line x1="9" y1="9" x2="15" y2="15"></line>
          </svg>
        ) : (
          <svg
            xmlns="http://www.w3.org/2000/svg"
            width="24"
            height="24"
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
          >
            <circle cx="11" cy="11" r="8"></circle>
            <path d="M21 21l-4.35-4.35"></path>
            <path d="M21 11A8 8 0 1 1 11 1z"></path>
          </svg>
        )}
      </button>

      {/* Chatbot panel */}
      {isOpen && (
        <div className="chatbot-panel">
          <div className="chatbot-header">
            <h3>Physical AI Assistant</h3>
            <p>Textbook-focused assistant (Physical AI, ROS 2, Simulation, Isaac, VLA)</p>
          </div>

          <div className="chat-messages">
            {messages.map((message) => (
              <div
                key={message.id}
                className={`message ${message.sender}`}
              >
                <div className="message-content">
                  <p>{message.text}</p>
                  {message.sources && message.sources.length > 0 && (
                    <div className="sources">
                      <p>Sources:</p>
                      <ul>
                        {message.sources.map((source, index) => (
                          <li key={index}>
                            <a href={source.url} target="_blank" rel="noopener noreferrer">
                              {source.title}
                            </a>
                          </li>
                        ))}
                      </ul>
                    </div>
                  )}
                </div>
                <small className="timestamp">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </small>
              </div>
            ))}
            {isLoading && (
              <div className="message bot">
                <div className="message-content">
                  <div className="typing-indicator">
                    <div></div>
                    <div></div>
                    <div></div>
                  </div>
                </div>
              </div>
            )}
            <div ref={messagesEndRef} />
          </div>

          <div className="chat-input-area">
            <textarea
              value={inputText}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the textbook (Physical AI, ROS 2, Gazebo, Isaac, VLA, etc.)..."
              rows="1"
              disabled={isLoading}
            />
            <button
              onClick={handleSendMessage}
              disabled={!inputText.trim() || isLoading}
              aria-label="Send message"
            >
              <svg
                xmlns="http://www.w3.org/2000/svg"
                width="20"
                height="20"
                viewBox="0 0 24 24"
                fill="none"
                stroke="currentColor"
                strokeWidth="2"
                strokeLinecap="round"
                strokeLinejoin="round"
              >
                <line x1="22" y1="2" x2="11" y2="13"></line>
                <polygon points="22 2 15 22 11 13 2 9 22 2"></polygon>
              </svg>
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default RAGChatbot;