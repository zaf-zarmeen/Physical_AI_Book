# API Contracts: Physical AI & Humanoid Robotics Textbook

## Interactive Features API

### Authentication Endpoints

#### POST /api/auth/login
Login user to track progress and enable personalization

**Request:**
```json
{
  "email": "string",
  "password": "string"
}
```

**Response:**
```json
{
  "success": "boolean",
  "user": {
    "id": "string",
    "email": "string",
    "name": "string"
  },
  "token": "string"
}
```

#### POST /api/auth/register
Register new user for the textbook platform

**Request:**
```json
{
  "name": "string",
  "email": "string",
  "password": "string"
}
```

**Response:**
```json
{
  "success": "boolean",
  "user": {
    "id": "string",
    "email": "string",
    "name": "string"
  }
}
```

### Progress Tracking Endpoints

#### GET /api/progress/{userId}
Get user's progress across the textbook

**Response:**
```json
{
  "userId": "string",
  "overallProgress": "number",
  "modules": [
    {
      "moduleId": "string",
      "moduleName": "string",
      "progress": "number",
      "chapters": [
        {
          "chapterId": "string",
          "chapterName": "string",
          "status": "string",
          "progress": "number"
        }
      ]
    }
  ]
}
```

#### PUT /api/progress/{userId}/chapter/{chapterId}
Update user's progress for a specific chapter

**Request:**
```json
{
  "status": "string",
  "progress": "number"
}
```

**Response:**
```json
{
  "success": "boolean",
  "updatedProgress": {
    "userId": "string",
    "chapterId": "string",
    "status": "string",
    "progress": "number",
    "updatedAt": "datetime"
  }
}
```

### RAG Chatbot Endpoints

#### POST /api/chatbot/query
Submit a query to the RAG-enhanced chatbot

**Request:**
```json
{
  "query": "string",
  "userId": "string (optional)",
  "context": {
    "currentChapter": "string",
    "userLevel": "string"
  }
}
```

**Response:**
```json
{
  "answer": "string",
  "sources": [
    {
      "title": "string",
      "url": "string",
      "relevance": "number"
    }
  ],
  "followupQuestions": [
    "string"
  ]
}
```

### Personalization Endpoints

#### GET /api/user/settings/{userId}
Get user's personalization settings

**Response:**
```json
{
  "userId": "string",
  "language": "string",
  "theme": "string",
  "fontSize": "string",
  "customizations": {
    "key": "value"
  }
}
```

#### PUT /api/user/settings/{userId}
Update user's personalization settings

**Request:**
```json
{
  "language": "string",
  "theme": "string",
  "fontSize": "string",
  "customizations": {
    "key": "value"
  }
}
```

**Response:**
```json
{
  "success": "boolean",
  "updatedSettings": {
    "userId": "string",
    "language": "string",
    "theme": "string",
    "fontSize": "string",
    "customizations": {
      "key": "value"
    }
  }
}
```

### Content Endpoints

#### GET /api/content/chapter/{chapterId}
Get content for a specific chapter with all associated materials

**Response:**
```json
{
  "id": "string",
  "title": "string",
  "content": "string (Markdown)",
  "module": "string",
  "order": "number",
  "learningObjectives": [
    "string"
  ],
  "codeExamples": [
    {
      "id": "string",
      "language": "string",
      "platform": "string",
      "code": "string",
      "description": "string"
    }
  ],
  "diagrams": [
    {
      "id": "string",
      "type": "string",
      "title": "string",
      "description": "string",
      "imagePath": "string",
      "textDescription": "string"
    }
  ],
  "exercises": [
    {
      "id": "string",
      "title": "string",
      "description": "string",
      "difficulty": "string",
      "type": "string"
    }
  ],
  "miniProjects": [
    {
      "id": "string",
      "title": "string",
      "description": "string",
      "requirements": [
        "string"
      ],
      "estimatedTime": "number"
    }
  ]
}
```

#### GET /api/content/search
Search textbook content

**Query Parameters:**
- q: search query string
- limit: number of results (default 10)
- module: filter by module (optional)

**Response:**
```json
{
  "query": "string",
  "results": [
    {
      "id": "string",
      "title": "string",
      "contentPreview": "string",
      "url": "string",
      "module": "string",
      "relevance": "number"
    }
  ],
  "totalResults": "number"
}
```

### Quiz and Project Submission Endpoints

#### POST /api/submissions/quiz
Submit answers to an exercise/quiz

**Request:**
```json
{
  "userId": "string",
  "exerciseId": "string",
  "answers": [
    {
      "questionId": "string",
      "answer": "string"
    }
  ]
}
```

**Response:**
```json
{
  "success": "boolean",
  "score": "number",
  "feedback": [
    {
      "questionId": "string",
      "isCorrect": "boolean",
      "explanation": "string"
    }
  ]
}
```

#### POST /api/submissions/project
Submit a mini project

**Request:**
```json
{
  "userId": "string",
  "miniProjectId": "string",
  "submissionUrl": "string",
  "reflection": "string"
}
```

**Response:**
```json
{
  "success": "boolean",
  "submissionId": "string",
  "submittedAt": "datetime"
}
```