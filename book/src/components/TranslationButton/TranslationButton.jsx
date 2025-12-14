import React, { useState, useEffect } from 'react';

/**
 * TranslationButton Component
 * Provides language translation functionality for the Physical AI textbook
 */
const TranslationButton = () => {
  const [currentLanguage, setCurrentLanguage] = useState('en');
  const [isDropdownOpen, setIsDropdownOpen] = useState(false);
  const [supportedLanguages, setSupportedLanguages] = useState([
    { code: 'en', name: 'English', flag: '🇺🇸' },
    { code: 'ur', name: 'Urdu', flag: '🇵🇰' },
    // Additional languages can be added here
  ]);

  // Detect user's preferred language
  useEffect(() => {
    const userLang = navigator.language || navigator.languages[0];
    const langCode = userLang.split('-')[0]; // Get the language code without region
    
    if (supportedLanguages.some(lang => lang.code === langCode)) {
      setCurrentLanguage(langCode);
    }
  }, []);

  const handleLanguageChange = (langCode) => {
    setCurrentLanguage(langCode);
    setIsDropdownOpen(false);
    
    // Save user's language preference
    localStorage.setItem('physicalAI_language', langCode);
    
    // In a real implementation, this would trigger:
    // 1. Loading of appropriate translation files
    // 2. Re-rendering of all text content
    // 3. Updating of any language-specific resources
    console.log(`Language changed to: ${langCode}`);
  };

  const toggleDropdown = () => {
    setIsDropdownOpen(!isDropdownOpen);
  };

  // Get the current language name and flag
  const currentLang = supportedLanguages.find(lang => lang.code === currentLanguage);

  return (
    <div className="translation-container">
      <button 
        className="translation-button"
        onClick={toggleDropdown}
        aria-expanded={isDropdownOpen}
        aria-label={`Change language, current: ${currentLang?.name}`}
      >
        <span className="flag">{currentLang?.flag}</span>
        <span className="lang-code">{currentLanguage.toUpperCase()}</span>
        <svg 
          xmlns="http://www.w3.org/2000/svg" 
          width="16" 
          height="16" 
          viewBox="0 0 24 24" 
          fill="none" 
          stroke="currentColor" 
          strokeWidth="2" 
          strokeLinecap="round" 
          strokeLinejoin="round"
          className={`dropdown-arrow ${isDropdownOpen ? 'rotated' : ''}`}
        >
          <polyline points="6 9 12 15 18 9"></polyline>
        </svg>
      </button>

      {isDropdownOpen && (
        <div className="translation-dropdown">
          <div className="dropdown-header">
            <h4>Select Language</h4>
          </div>
          <ul className="language-list">
            {supportedLanguages.map((lang) => (
              <li key={lang.code}>
                <button
                  className={`language-option ${currentLanguage === lang.code ? 'selected' : ''}`}
                  onClick={() => handleLanguageChange(lang.code)}
                  aria-current={currentLanguage === lang.code}
                >
                  <span className="flag">{lang.flag}</span>
                  <span className="lang-name">{lang.name}</span>
                  {currentLanguage === lang.code && (
                    <span className="checkmark">✓</span>
                  )}
                </button>
              </li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
};

export default TranslationButton;