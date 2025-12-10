---

description: "Task list for Physical AI & Humanoid Robotics Textbook"
---

# Tasks: Physical AI & Humanoid Robotics Textbook

**Input**: Design documents from `/specs/1-create-physical-ai-book/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, data-model.md, contracts/

**Tests**: The examples below include test tasks. Tests are OPTIONAL - only include them if explicitly requested in the feature specification.

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Single project**: `src/`, `tests/` at repository root
- **Web app**: `backend/src/`, `frontend/src/`
- **Mobile**: `api/src/`, `ios/src/` or `android/src/`
- Paths shown below assume single project - adjust based on plan.md structure

<!--
  ============================================================================
  IMPORTANT: The tasks below are SAMPLE TASKS for illustration purposes only.

  The /sp.tasks command MUST replace these with actual tasks based on:
  - User stories from spec.md (with their priorities P1, P2, P3...)
  - Feature requirements from plan.md
  - Entities from data-model.md
  - Endpoints from contracts/

  Tasks MUST be organized by user story so each story can be:
  - Implemented independently
  - Tested independently
  - Delivered as an MVP increment

  DO NOT keep these sample tasks in the generated tasks.md file.
  ============================================================================
-->

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Project initialization and basic structure

- [ ] T001 Create project structure per implementation plan in book/
- [ ] T002 [P] Initialize Docusaurus project with book/package.json dependencies
- [ ] T003 [P] Configure sidebar and navigation in book/sidebars.js
- [ ] T004 [P] Deploy to GitHub Pages in book/docusaurus.config.js

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before ANY user story can be implemented

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

Examples of foundational tasks (adjust based on your project):

- [ ] T005 Setup basic Docusaurus configuration in book/docusaurus.config.js
- [ ] T006 [P] Create directory structure for textbook modules in book/docs/
- [ ] T007 [P] Create basic React components directory in book/src/components/
- [ ] T008 [P] Setup example code directories in book/examples/
- [ ] T009 Create static assets directory in book/static/
- [ ] T010 Configure internationalization for Urdu translation in book/docusaurus.config.js

**Checkpoint**: Foundation ready - user story implementation can now begin in parallel

---

## Phase 3: User Story 1 - Student Learns Physical AI Concepts (Priority: P1) 🎯 MVP

**Goal**: Students can read the first 3 chapters and successfully implement the associated ROS 2 code examples in Gazebo simulation, demonstrating understanding of core Physical AI concepts.

**Independent Test**: Students can read the first 3 chapters and successfully implement the associated ROS 2 code examples in Gazebo simulation, demonstrating understanding of core Physical AI concepts.

### Tests for User Story 1 (OPTIONAL - only if tests requested) ⚠️

> **NOTE: Write these tests FIRST, ensure they FAIL before implementation**

- [ ] T011 [P] [US1] Contract test for GET /api/content/chapter/{chapterId} in tests/contract/test_content.py
- [ ] T012 [P] [US1] Integration test for student ROS 2 chapter implementation in tests/integration/test_ros2_chapter.py

### Implementation for User Story 1

- [ ] T013 [P] [US1] Create Chapter model for textbook content in book/src/models/chapter.py
- [ ] T014 [P] [US1] Create CodeExample model in book/src/models/code_example.py
- [ ] T015 [P] [US1] Create Diagram model in book/src/models/diagram.py
- [ ] T016 [US1] Create Intro to Physical AI chapter in book/docs/intro/intro-to-physical-ai.md
- [ ] T017 [US1] Create ROS 2 fundamentals chapter in book/docs/ros2/ros2-fundamentals.md
- [ ] T018 [US1] Create Gazebo physics chapter in book/docs/simulation/gazebo-physics.md
- [ ] T019 [US1] Add ROS 2 Python code examples in book/examples/ros2/fundamentals/
- [ ] T020 [US1] Add Gazebo simulation examples in book/examples/gazebo/fundamentals/
- [ ] T021 [US1] Create exercise models for chapter assessments in book/src/models/exercise.py
- [ ] T022 [US1] Add exercises for each chapter in book/docs/[module]/[chapter].md
- [ ] T023 [US1] Create mini project models in book/src/models/mini_project.py
- [ ] T024 [US1] Add mini projects for each chapter in book/docs/[module]/[chapter].md
- [ ] T025 [US1] Add text-based diagrams for each chapter in book/docs/[module]/[chapter].md

**Checkpoint**: At this point, User Story 1 should be fully functional and testable independently

---

## Phase 4: User Story 2 - Instructor Uses Textbook for Course Delivery (Priority: P2)

**Goal**: An instructor can create a full 13-week syllabus using the textbook, with appropriate assignments, projects, and assessments for each module.

**Independent Test**: An instructor can create a full 13-week syllabus using the textbook, with appropriate assignments, projects, and assessments for each module.

### Tests for User Story 2 (OPTIONAL - only if tests requested) ⚠️

- [ ] T026 [P] [US2] Contract test for GET /api/content/search in tests/contract/test_search.py
- [ ] T027 [P] [US2] Integration test for syllabus creation workflow in tests/integration/test_syllabus.py

### Implementation for User Story 2

- [ ] T028 [P] [US2] Create Module entity in book/src/models/module.py
- [ ] T029 [US2] Create advanced chapters for curriculum completion in book/docs/[module]/[advanced chapter].md
- [ ] T030 [US2] Enhance search functionality for instructor use in book/src/components/SearchBar/
- [ ] T031 [US2] Create syllabus creation tools in book/src/components/SyllabusBuilder/
- [ ] T032 [US2] Add learning objectives to each chapter in book/docs/[module]/[chapter].md
- [ ] T033 [US2] Create assessment tools for instructors in book/src/components/AssessmentTools/
- [ ] T034 [US2] Create detailed project requirements in book/docs/[module]/mini-projects.md

**Checkpoint**: At this point, User Stories 1 AND 2 should both work independently

---

## Phase 5: User Story 3 - Developer Implements Textbook Examples (Priority: P3)

**Goal**: A robotics developer can take examples from Module 3 (NVIDIA Isaac + SLAM + Nav2) and implement similar navigation systems in their own projects.

**Independent Test**: A robotics developer can take examples from Module 3 (NVIDIA Isaac + SLAM + Nav2) and implement similar navigation systems in their own projects.

### Tests for User Story 3 (OPTIONAL - only if tests requested) ⚠️

- [ ] T035 [P] [US3] Contract test for Isaac Sim workflows in tests/contract/test_workflows.py
- [ ] T036 [P] [US3] Integration test for Isaac+SLAM+Nav2 implementation in tests/integration/test_isaac_nav.py

### Implementation for User Story 3

- [ ] T037 [P] [US3] Create Isaac Sim basics chapter in book/docs/isaac/isaac-sim-basics.md
- [ ] T038 [P] [US3] Create Isaac ROS + SLAM chapter in book/docs/isaac/isaac-ros-slam.md
- [ ] T039 [P] [US3] Create Nav2 locomotion chapter in book/docs/isaac/nav2-locomotion.md
- [ ] T040 [US3] Add Isaac Sim code examples in book/examples/isaac/basics/
- [ ] T041 [US3] Add SLAM code examples in book/examples/isaac/slam/
- [ ] T042 [US3] Add Nav2 code examples in book/examples/isaac/nav2/
- [ ] T043 [US3] Add Unity visualization chapter in book/docs/simulation/unity-visualization.md
- [ ] T044 [US3] Add Unity code examples in book/examples/unity/
- [ ] T045 [US3] Add URDF chapter for humanoids in book/docs/ros2/urdf-humanoids.md
- [ ] T046 [US3] Add URDF examples in book/examples/ros2/urdf/

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: User Story 4 - User Accesses Interactive Features (Priority: P4)

**Goal**: A user can access the Urdu translation feature and successfully navigate the textbook content in their preferred language.

**Independent Test**: A user can access the Urdu translation feature and successfully navigate the textbook content in their preferred language.

### Tests for User Story 4 (OPTIONAL - only if tests requested) ⚠️

- [ ] T047 [P] [US4] Contract test for personalization settings in tests/contract/test_personalization.py
- [ ] T048 [P] [US4] Contract test for translation endpoints in tests/contract/test_translation.py

### Implementation for User Story 4

- [ ] T049 [P] [US4] Create Student entity in book/src/models/student.py
- [ ] T050 [P] [US4] Create UserSetting entity in book/src/models/user_setting.py
- [ ] T051 [P] [US4] Create ProgressItem entity in book/src/models/progress_item.py
- [ ] T052 [US4] Implement Better-Auth signup/login functionality in book/src/components/BetterAuth/
- [ ] T053 [US4] Create personalization button component in book/src/components/PersonalizationButton/
- [ ] T054 [US4] Create Urdu translation button component in book/src/components/TranslationButton/
- [ ] T055 [US4] Implement RAG chatbot system in book/src/components/RAGChatbot/
- [ ] T056 [US4] Create API endpoints for progress tracking in book/src/api/progress.js
- [ ] T057 [US4] Create API endpoints for user settings in book/src/api/settings.js
- [ ] T058 [US4] Create API endpoints for content delivery in book/src/api/content.js
- [ ] T059 [US4] Create VLA (Whisper + GPT planner) chapter in book/docs/vla-ai/vla-implementation.md
- [ ] T060 [US4] Add VLA code examples in book/examples/vla/
- [ ] T061 [US4] Configure Urdu localization files in book/i18n/
- [ ] T062 [US4] Implement Urdu translation support in book/docusaurus.config.js

---

## Phase 7: Capstone Project

**Goal**: Complete the final capstone project that integrates all concepts from the textbook

- [ ] T063 Create capstone project chapter in book/docs/capstone/final-capstone.md
- [ ] T064 Implement capstone project requirements in book/docs/capstone/project-requirements.md
- [ ] T065 Create capstone project code examples in book/examples/capstone/
- [ ] T066 Add capstone project exercises in book/docs/capstone/exercises.md
- [ ] T067 Create submission system for capstone projects in book/src/components/ProjectSubmission/

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Improvements that affect multiple user stories

- [ ] T068 [P] Documentation updates in book/docs/
- [ ] T069 [P] Add sim-to-real notes in book/docs/appendices/sim-to-real-notes.md
- [ ] T070 [P] Add hardware setup guides in book/docs/appendices/hardware-setup.md
- [ ] T071 [P] Add GPU workstation requirements in book/docs/appendices/gpu-requirements.md
- [ ] T072 [P] Code cleanup and refactoring
- [ ] T073 Performance optimization across all stories
- [ ] T074 [P] Additional unit tests (if requested) in book/tests/unit/
- [ ] T075 Security hardening
- [ ] T076 Run quickstart.md validation

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Setup completion - BLOCKS all user stories
- **User Stories (Phase 3+)**: All depend on Foundational phase completion
  - User stories can then proceed in parallel (if staffed)
  - Or sequentially in priority order (P1 → P2 → P3 → P4)
- **Capstone (Phase 7)**: Depends on all core modules being complete
- **Polish (Final Phase)**: Depends on all desired user stories and capstone being complete

### User Story Dependencies

- **User Story 1 (P1)**: Can start after Foundational (Phase 2) - No dependencies on other stories
- **User Story 2 (P2)**: Can start after Foundational (Phase 2) - May integrate with US1 but should be independently testable
- **User Story 3 (P3)**: Can start after Foundational (Phase 2) - May integrate with US1/US2 but should be independently testable
- **User Story 4 (P4)**: Can start after Foundational (Phase 2) - May integrate with earlier stories but should be independently testable

### Within Each User Story

- Tests (if included) MUST be written and FAIL before implementation
- Models before services
- Services before endpoints
- Core implementation before integration
- Story complete before moving to next priority

### Parallel Opportunities

- All Setup tasks marked [P] can run in parallel
- All Foundational tasks marked [P] can run in parallel (within Phase 2)
- Once Foundational phase completes, all user stories can start in parallel (if team capacity allows)
- All tests for a user story marked [P] can run in parallel
- Models within a story marked [P] can run in parallel
- Different user stories can be worked on in parallel by different team members

---

## Parallel Example: User Story 1

```bash
# Launch all models for User Story 1 together:
Task: "Create Chapter model for textbook content in book/src/models/chapter.py"
Task: "Create CodeExample model in book/src/models/code_example.py"
Task: "Create Diagram model in book/src/models/diagram.py"
Task: "Create exercise models for chapter assessments in book/src/models/exercise.py"
Task: "Create mini project models in book/src/models/mini_project.py"

# Launch all chapter creation together:
Task: "Create Intro to Physical AI chapter in book/docs/intro/intro-to-physical-ai.md"
Task: "Create ROS 2 fundamentals chapter in book/docs/ros2/ros2-fundamentals.md"
Task: "Create Gazebo physics chapter in book/docs/simulation/gazebo-physics.md"
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (CRITICAL - blocks all stories)
3. Complete Phase 3: User Story 1
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready

### Incremental Delivery

1. Complete Setup + Foundational → Foundation ready
2. Add User Story 1 → Test independently → Deploy/Demo (MVP!)
3. Add User Story 2 → Test independently → Deploy/Demo
4. Add User Story 3 → Test independently → Deploy/Demo
5. Add User Story 4 → Test independently → Deploy/Demo
6. Add Capstone → Test independently → Deploy/Demo (Complete textbook!)
7. Each story adds value without breaking previous stories

### Parallel Team Strategy

With multiple developers:

1. Team completes Setup + Foundational together
2. Once Foundational is done:
   - Developer A: User Story 1
   - Developer B: User Story 2
   - Developer C: User Story 3
   - Developer D: User Story 4
3. Stories complete and integrate independently

---

## Notes

- [P] tasks = different files, no dependencies
- [Story] label maps task to specific user story for traceability
- Each user story should be independently completable and testable
- Verify tests fail before implementing
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- Avoid: vague tasks, same file conflicts, cross-story dependencies that break independence