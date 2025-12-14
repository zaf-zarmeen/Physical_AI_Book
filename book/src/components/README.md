# Interactive Components for Physical AI & Humanoid Robotics Textbook

This directory contains interactive React components that enhance the learning experience in the Physical AI & Humanoid Robotics textbook. These components are referenced throughout the textbook and provide hands-on interaction with key concepts.

## Components Overview

### PersonalizationButton
A React component that allows readers to customize their learning experience with:
- Theme selection (light/dark/auto)
- Font size adjustment
- Language preferences
- Learning pace settings
- Notification preferences

### TranslationButton
Provides language translation functionality with:
- Support for Urdu and English (expandable to other languages)
- Flag indicators for language selection
- Browser language detection
- Local storage for preference persistence

### RAGChatbot
A Retrieval-Augmented Generation chatbot that:
- Answers questions about Physical AI concepts
- References specific chapters and sections in the textbook
- Provides context-aware responses based on the textbook content
- Simulates AI-powered learning assistance
- Only responds to questions related to the textbook content
- If asked non-textbook questions, responds with "I can only answer questions related to the Physical AI & Humanoid Robotics textbook..."

### BetterAuth Integration
Placeholder for user authentication functionality that would include:
- User registration and login
- Progress tracking
- Personalization settings persistence
- Content access control

## Usage in Docusaurus

These components are designed to be integrated into the Docusaurus-based textbook. To use a component in a markdown page:

```jsx
import PersonalizationButton from '@site/src/components/PersonalizationButton/PersonalizationButton';

<PersonalizationButton />
```

## Implementation Notes

- Components are designed to be self-contained with minimal dependencies
- All components use localStorage for preference persistence
- Components follow accessibility best practices (ARIA labels, keyboard navigation)
- The RAGChatbot simulates API interactions with a mock response system
- Components are styled to match the textbook's visual design

## Future Enhancements

- Integration with actual RAG API backend
- More sophisticated personalization options
- Multi-language support beyond Urdu
- Integration with learning management systems
- Analytics tracking for educational insights

These components represent the interactive features mentioned in the textbook's requirements, providing students with a more engaging learning experience that adapts to their needs.