import React, { useState, useEffect, useContext } from 'react';
import styles from './styles.module.css'; // Assuming a CSS module for styling
import { v4 as uuidv4 } from 'uuid'; // For generating unique session/message IDs
import { useSelectedText } from '@site/src/contexts/SelectedTextContext'; // Import the hook
import useDocusaurusContext from '@docusaurus/useDocusaurusContext'; // Import useDocusaurusContext

interface ChatMessage {
  message_id: string;
  session_id: string;
  sender: 'user' | 'assistant';
  text: string;
  timestamp: string;
  context_used?: string[];
}

interface ChatRequest {
  session_id?: string;
  user_id: string;
  message: string;
  mode: 'full_book' | 'selection';
  selected_text?: string | null;
}

interface ChatResponse {
  session_id: string;
  message_id: string;
  response: string;
  timestamp: string;
  context_used?: string[];
}

const ChatbotWidget: React.FC = () => {
  const {
    siteConfig: { customFields },
  } = useDocusaurusContext();
  const CHATBOT_API_BASE_URL = customFields.CHATBOT_API_BASE_URL as string;

  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [sessionId, setSessionId] = useState<string | undefined>(undefined);
  const [userId, setUserId] = useState<string>(''); // Anonymous user ID
  const [chatMode, setChatMode] = useState<'full_book' | 'selection'>('full_book'); // New state for chat mode

  const { selectedText, setSelectedText } = useSelectedText(); // Use the hook

  useEffect(() => {
    // Initialize or retrieve anonymous user ID from local storage
    let storedUserId = localStorage.getItem('chatbot_user_id');
    if (!storedUserId) {
      storedUserId = uuidv4();
      localStorage.setItem('chatbot_user_id', storedUserId);
    }
    setUserId(storedUserId);

    // Try to get existing session ID
    const storedSessionId = localStorage.getItem(`chatbot_session_id_${storedUserId}`);
    if (storedSessionId) {
      setSessionId(storedSessionId);
    }
  }, []);

  useEffect(() => {
    const fetchChatHistory = async () => {
      if (isOpen && sessionId) {
        try {
          const response = await fetch(`${CHATBOT_API_BASE_URL}/history/${sessionId}`);
          if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
          }
          const data: ChatMessage[] = await response.json();
          setMessages(data);
        } catch (error) {
          console.error('Error fetching chat history:', error);
          setMessages([]); // Clear messages on error
        }
      }
    };

    fetchChatHistory();
  }, [isOpen, sessionId]); // Refetch when chat opens or session ID changes

  const toggleChat = () => {
    setIsOpen(!isOpen);
    // Clear selected text when closing chat
    if (isOpen) {
      setSelectedText(null);
    }
  };

  const handleInputChange = (event: React.ChangeEvent<HTMLInputElement>) => {
    setInputValue(event.target.value);
  };

  const sendMessage = async () => {
    if (inputValue.trim() === '') return;

    // If in selection mode but no text is selected, prevent sending
    if (chatMode === 'selection' && !selectedText) {
      alert('Please select some text in the book before asking a question in selection mode.');
      return;
    }

    const newUserMessage: ChatMessage = {
      message_id: uuidv4(),
      session_id: sessionId || '', // Will be updated by backend if new session
      sender: 'user',
      text: inputValue,
      timestamp: new Date().toISOString(),
    };
    setMessages((prevMessages) => [...prevMessages, newUserMessage]);

    const requestBody: ChatRequest = {
      session_id: sessionId,
      user_id: userId,
      message: inputValue,
      mode: chatMode,
      selected_text: chatMode === 'selection' ? selectedText : null,
    };

    try {
      const response = await fetch(CHATBOT_API_BASE_URL, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(requestBody),
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      const data: ChatResponse = await response.json();
      
      // Update session_id if it's the first message
      if (!sessionId) {
        setSessionId(data.session_id);
        localStorage.setItem(`chatbot_session_id_${userId}`, data.session_id); // Store new session ID
      }

      const assistantMessage: ChatMessage = {
        message_id: data.message_id,
        session_id: data.session_id,
        sender: 'assistant',
        text: data.response,
        timestamp: data.timestamp,
        context_used: data.context_used,
      };
      setMessages((prevMessages) => [...prevMessages, assistantMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      const errorMessage: ChatMessage = {
        message_id: uuidv4(),
        session_id: sessionId || '',
        sender: 'assistant',
        text: 'Sorry, I am having trouble connecting. Please try again later.',
        timestamp: new Date().toISOString(),
      };
      setMessages((prevMessages) => [...prevMessages, errorMessage]);
    }

    setInputValue('');
    setSelectedText(null); // Clear selected text after sending message
  };

  const handleKeyPress = (event: React.KeyboardEvent<HTMLInputElement>) => {
    if (event.key === 'Enter') {
      sendMessage();
    }
  };

  const clearChatHistory = async () => {
    if (!sessionId) return;

    try {
      const response = await fetch(`${CHATBOT_API_BASE_URL}/reset/${sessionId}`, {
        method: 'POST',
      });

      if (!response.ok) {
        throw new Error(`HTTP error! status: ${response.status}`);
      }

      setMessages([]); // Clear local messages
      setSessionId(undefined); // Clear session ID
      localStorage.removeItem(`chatbot_session_id_${userId}`); // Remove from local storage
      alert('Chat history cleared!');
    } catch (error) {
      console.error('Error clearing chat history:', error);
      alert('Failed to clear chat history.');
    }
  };

  return (
    <div className={styles.chatbotContainer}>
      <button className={styles.chatbotToggle} onClick={toggleChat}>
        {isOpen ? 'Close Chat' : 'Open Chat'}
      </button>
      {isOpen && (
        <div className={styles.chatWindow}>
          <div className={styles.chatHeader}>
            <h3>Book Chatbot</h3>
            <div className={styles.modeToggle}>
              <button
                className={`${styles.modeButton} ${chatMode === 'full_book' ? styles.activeMode : ''}`}
                onClick={() => {
                  setChatMode('full_book');
                  setSelectedText(null); // Clear selected text when switching mode
                }}
              >
                Full Book
              </button>
              <button
                className={`${styles.modeButton} ${chatMode === 'selection' ? styles.activeMode : ''}`}
                onClick={() => setChatMode('selection')}
              >
                Selection
              </button>
              <button className={styles.modeButton} onClick={clearChatHistory}>
                Clear History
              </button>
            </div>
            <button onClick={toggleChat}>X</button>
          </div>
          <div className={styles.chatMessages}>
            {messages.map((msg, index) => (
              <div key={index} className={`${styles.message} ${styles[msg.sender]}`}>
                <p><strong>{msg.sender === 'user' ? 'You' : 'Bot'}:</strong> {msg.text}</p>
                {msg.context_used && msg.context_used.length > 0 && (
                  <small>Context: {msg.context_used.join(', ')}</small>
                )}
              </div>
            ))}
          </div>
          <div className={styles.chatInput}>
            {chatMode === 'selection' && selectedText && (
              <div className={styles.selectedTextDisplay}>
                Selected: "{selectedText.substring(0, 50)}..."
              </div>
            )}
            <input
              type="text"
              value={inputValue}
              onChange={handleInputChange}
              onKeyPress={handleKeyPress}
              placeholder="Ask a question about the book..."
            />
            <button onClick={sendMessage}>Send</button>
          </div>
        </div>
      )}
    </div>
  );
};

export default ChatbotWidget;
