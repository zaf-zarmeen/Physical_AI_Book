import React, { useState, useEffect } from 'react';

/**
 * PersonalizationButton Component
 * Allows users to customize their learning experience in the Physical AI textbook
 */
const PersonalizationButton = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [settings, setSettings] = useState({
    theme: 'light', // 'light', 'dark', 'auto'
    fontSize: 'medium', // 'small', 'medium', 'large'
    language: 'en', // 'en', 'ur', etc.
    learningPace: 'moderate', // 'slow', 'moderate', 'fast'
    notificationPreferences: {
      progressUpdates: true,
      newContent: true,
      reminders: false
    }
  });

  // Load saved settings from localStorage on component mount
  useEffect(() => {
    const savedSettings = localStorage.getItem('physicalAI_personalization');
    if (savedSettings) {
      setSettings(JSON.parse(savedSettings));
    }
  }, []);

  // Save settings to localStorage whenever they change
  useEffect(() => {
    localStorage.setItem('physicalAI_personalization', JSON.stringify(settings));
    
    // Apply theme to document
    document.documentElement.setAttribute('data-theme', settings.theme);
    
    // Apply font size to document
    document.documentElement.style.fontSize = 
      settings.fontSize === 'small' ? '14px' :
      settings.fontSize === 'large' ? '18px' : '16px';
  }, [settings]);

  const handleSettingChange = (settingName, value) => {
    setSettings(prev => ({
      ...prev,
      [settingName]: value
    }));
  };

  const toggleNotification = (notificationType) => {
    setSettings(prev => ({
      ...prev,
      notificationPreferences: {
        ...prev.notificationPreferences,
        [notificationType]: !prev.notificationPreferences[notificationType]
      }
    }));
  };

  const toggleMenu = () => {
    setIsOpen(!isOpen);
  };

  return (
    <div className="personalization-container">
      <button 
        className="personalization-button"
        onClick={toggleMenu}
        aria-expanded={isOpen}
        aria-label="Personalization settings"
      >
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
          <circle cx="12" cy="12" r="3"></circle>
          <path d="M19.4 15a1.65 1.65 0 0 0 .33 1.82l.06.06a2 2 0 0 1 0 2.83 2 2 0 0 1-2.83 0l-.06-.06a1.65 1.65 0 0 0-1.82-.33 1.65 1.65 0 0 0-1 1.51V21a2 2 0 0 1-2 2 2 2 0 0 1-2-2v-.09A1.65 1.65 0 0 0 9 19.4a1.65 1.65 0 0 0-1.82.33l-.06.06a2 2 0 0 1-2.83 0 2 2 0 0 1 0-2.83l.06-.06a1.65 1.65 0 0 0 .33-1.82 1.65 1.65 0 0 0-1.51-1H3a2 2 0 0 1-2-2 2 2 0 0 1 2-2h.09A1.65 1.65 0 0 0 4.6 9a1.65 1.65 0 0 0-.33-1.82l-.06-.06a2 2 0 0 1 0-2.83 2 2 0 0 1 2.83 0l.06.06a1.65 1.65 0 0 0 1.82.33H9a1.65 1.65 0 0 0 1-1.51V3a2 2 0 0 1 2-2 2 2 0 0 1 2 2v.09a1.65 1.65 0 0 0 1 1.51 1.65 1.65 0 0 0 1.82-.33l.06-.06a2 2 0 0 1 2.83 0 2 2 0 0 1 0 2.83l-.06.06a1.65 1.65 0 0 0-.33 1.82V9a1.65 1.65 0 0 0 1.51 1H21a2 2 0 0 1 2 2 2 2 0 0 1-2 2h-.09a1.65 1.65 0 0 0-1.51 1z"></path>
        </svg>
      </button>

      {isOpen && (
        <div className="personalization-panel">
          <div className="panel-header">
            <h3>Personalization Settings</h3>
            <button 
              className="close-button" 
              onClick={toggleMenu}
              aria-label="Close settings"
            >
              ×
            </button>
          </div>
          
          <div className="panel-content">
            <div className="setting-group">
              <label htmlFor="theme-select">Theme:</label>
              <select
                id="theme-select"
                value={settings.theme}
                onChange={(e) => handleSettingChange('theme', e.target.value)}
              >
                <option value="light">Light</option>
                <option value="dark">Dark</option>
                <option value="auto">Auto</option>
              </select>
            </div>
            
            <div className="setting-group">
              <label htmlFor="font-size-select">Font Size:</label>
              <select
                id="font-size-select"
                value={settings.fontSize}
                onChange={(e) => handleSettingChange('fontSize', e.target.value)}
              >
                <option value="small">Small</option>
                <option value="medium">Medium</option>
                <option value="large">Large</option>
              </select>
            </div>
            
            <div className="setting-group">
              <label htmlFor="language-select">Language:</label>
              <select
                id="language-select"
                value={settings.language}
                onChange={(e) => handleSettingChange('language', e.target.value)}
              >
                <option value="en">English</option>
                <option value="ur">Urdu</option>
              </select>
            </div>
            
            <div className="setting-group">
              <label htmlFor="pace-select">Learning Pace:</label>
              <select
                id="pace-select"
                value={settings.learningPace}
                onChange={(e) => handleSettingChange('learningPace', e.target.value)}
              >
                <option value="slow">Slow</option>
                <option value="moderate">Moderate</option>
                <option value="fast">Fast</option>
              </select>
            </div>
            
            <div className="setting-group">
              <h4>Notifications</h4>
              <div className="notification-option">
                <input
                  type="checkbox"
                  id="progress-updates"
                  checked={settings.notificationPreferences.progressUpdates}
                  onChange={() => toggleNotification('progressUpdates')}
                />
                <label htmlFor="progress-updates">Progress updates</label>
              </div>
              <div className="notification-option">
                <input
                  type="checkbox"
                  id="new-content"
                  checked={settings.notificationPreferences.newContent}
                  onChange={() => toggleNotification('newContent')}
                />
                <label htmlFor="new-content">New content alerts</label>
              </div>
              <div className="notification-option">
                <input
                  type="checkbox"
                  id="reminders"
                  checked={settings.notificationPreferences.reminders}
                  onChange={() => toggleNotification('reminders')}
                />
                <label htmlFor="reminders">Learning reminders</label>
              </div>
            </div>
          </div>
          
          <div className="panel-footer">
            <button 
              className="reset-button" 
              onClick={() => {
                setSettings({
                  theme: 'light',
                  fontSize: 'medium',
                  language: 'en',
                  learningPace: 'moderate',
                  notificationPreferences: {
                    progressUpdates: true,
                    newContent: true,
                    reminders: false
                  }
                });
              }}
            >
              Reset to defaults
            </button>
          </div>
        </div>
      )}
    </div>
  );
};

export default PersonalizationButton;