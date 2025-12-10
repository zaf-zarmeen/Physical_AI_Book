# Data Model: Physical AI & Humanoid Robotics Textbook

## Phase 1: Design & Contracts

### Entities and Data Structure

#### 1. Student
- **Fields**:
  - id: string (unique identifier)
  - name: string
  - email: string
  - progress: array of ProgressItem
  - preferences: object (personalization settings)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**: 
  - email must be valid email format
  - name must be non-empty
  - progress items must reference valid chapters/sections

- **Relationships**:
  - One-to-many with ProgressItem
  - One-to-many with QuizSubmission

#### 2. Chapter
- **Fields**:
  - id: string (unique identifier)
  - title: string
  - module: string (Module 1-4 or Capstone)
  - order: number (position in curriculum)
  - content: string (Markdown content)
  - codeExamples: array of CodeExample
  - diagrams: array of Diagram
  - exercises: array of Exercise
  - miniProjects: array of MiniProject
  - learningObjectives: array of string
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - title must be non-empty
  - module must be one of the 4 curriculum modules or capstone
  - order must be positive integer
  - content must be valid Markdown

- **Relationships**:
  - One-to-many with CodeExample, Diagram, Exercise, MiniProject
  - Many-to-many with Student (through ProgressItem)

#### 3. CodeExample
- **Fields**:
  - id: string (unique identifier)
  - language: string ("python", "javascript", etc.)
  - platform: string ("ros2", "gazebo", "unity", "isaac")
  - code: string
  - description: string
  - usage: string (how to run/use the example)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - language must be supported language
  - platform must be supported platform
  - code must be non-empty

- **Relationships**:
  - Belongs to one Chapter

#### 4. Diagram
- **Fields**:
  - id: string (unique identifier)
  - type: string ("uml", "flowchart", "architecture", "simulation", etc.)
  - title: string
  - description: string
  - imagePath: string
  - textDescription: string (for accessibility)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - title must be non-empty
  - imagePath must be valid path
  - textDescription required for accessibility

- **Relationships**:
  - Belongs to one Chapter

#### 5. Exercise
- **Fields**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - difficulty: string ("beginner", "intermediate", "advanced")
  - type: string ("multiple-choice", "coding", "simulation", "essay")
  - solution: string
  - hints: array of string
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - title must be non-empty
  - difficulty must be one of allowed values
  - type must be one of allowed values

- **Relationships**:
  - Belongs to one Chapter
  - One-to-many with QuizSubmission

#### 6. MiniProject
- **Fields**:
  - id: string (unique identifier)
  - title: string
  - description: string
  - requirements: array of string
  - estimatedTime: number (in hours)
  - deliverables: array of string
  - evaluationCriteria: array of string
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - title must be non-empty
  - estimatedTime must be positive number
  - requirements must be non-empty

- **Relationships**:
  - Belongs to one Chapter
  - One-to-many with ProjectSubmission

#### 7. ProgressItem
- **Fields**:
  - id: string (unique identifier)
  - studentId: string
  - chapterId: string
  - status: string ("not-started", "in-progress", "completed")
  - progressPercentage: number (0-100)
  - lastAccessed: datetime
  - completedAt: datetime (nullable)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - status must be one of allowed values
  - progressPercentage must be between 0-100

- **Relationships**:
  - Belongs to one Student
  - Belongs to one Chapter

#### 8. QuizSubmission
- **Fields**:
  - id: string (unique identifier)
  - studentId: string
  - exerciseId: string
  - answers: array of object
  - score: number (0-100)
  - submittedAt: datetime
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - score must be between 0-100
  - answers must match the exercise format

- **Relationships**:
  - Belongs to one Student
  - Belongs to one Exercise

#### 9. ProjectSubmission
- **Fields**:
  - id: string (unique identifier)
  - studentId: string
  - miniProjectId: string
  - submissionUrl: string
  - evaluation: string
  - score: number (0-100)
  - submittedAt: datetime
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - submissionUrl must be valid URL or file path
  - score must be between 0-100

- **Relationships**:
  - Belongs to one Student
  - Belongs to one MiniProject

#### 10. UserSetting
- **Fields**:
  - id: string (unique identifier)
  - studentId: string
  - language: string ("en", "ur", etc.)
  - theme: string ("light", "dark", "auto")
  - fontSize: string ("small", "medium", "large")
  - customizations: object (additional personalization options)
  - createdAt: datetime
  - updatedAt: datetime

- **Validation**:
  - language must be supported language
  - theme must be valid option
  - fontSize must be valid option

- **Relationships**:
  - Belongs to one Student

### State Transitions

#### Student Progress State Transitions
- "not-started" → "in-progress" (when student begins chapter)
- "in-progress" → "completed" (when student completes all requirements)
- "completed" → "in-progress" (if student returns to chapter for review)

#### Chapter Access State Transitions
- Unlocked → Locked (when prerequisite chapters are not completed)
- Locked → Unlocked (when prerequisites are met)