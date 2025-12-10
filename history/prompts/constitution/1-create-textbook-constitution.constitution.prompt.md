---
id: 1
title: Create textbook constitution
stage: constitution
date: 2025-12-09
surface: agent
model: gpt-4
feature: none
branch: main
user: UZM
command: /sp.constitution \"Physical AI & Humanoid Robotics textbook\"
labels: [\"constitution\",\"textbook\",\"ai\",\"robotics\"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - none
---

## Prompt

/sp.constitution \"\" You are an AI expert textbook creator. Your job is to write a complete, professional textbook titled:

“Physical AI & Humanoid Robotics”

Your writing must be:

Simple, clear, structured

Educator-level, engineering-grade

ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action based

According to the 13-week course outline

Ready for Docusaurus

Suitable for a RAG chatbot

Suitable for personalization and Urdu translation

You must act as:

Robotics teacher

AI engineer

Technical writer

Docusaurus documentation exper  \"\"\"

## Response snapshot

I've successfully created the constitution for your "Physical AI & Humanoid Robotics" textbook project with the following details:

- **New Version**: 1.0.0 (initial version)
- **Bump Rationale**: Initial creation of the constitution for the textbook project with principles tailored specifically for developing educational content focused on ROS 2, Gazebo, Unity, NVIDIA Isaac, and Vision-Language-Action paradigms
- **Files Flagged for Manual Follow-up**: None required
- **Suggested Commit Message**: `docs: create constitution for Physical AI & Humanoid Robotics textbook project v1.0.0`

The updated constitution includes six core principles that align with your requirements:
1. Educational Clarity
2. Multi-Platform Integration
3. Vision-Language-Action Foundation (NON-NEGOTIABLE)
4. 13-Week Course Alignment
5. Docusaurus Optimization
6. Accessibility and Localization

The constitution also includes specific sections for Technology Standards, Development Workflow, and Governance that reflect the unique requirements of creating a technical textbook in the field of Physical AI and Humanoid Robotics.

All placeholder values have been properly filled, and the file has been validated to ensure no remaining unexplained bracket tokens, correct ISO date formatting, and clear, testable principles with appropriate MUST/SHOULD language.

## Outcome

- ✅ Impact: Updated constitution file with appropriate principles for textbook project
- 🧪 Tests: none
- 📁 Files: .specify/memory/constitution.md updated
- 🔁 Next prompts: Next steps would be to begin textbook development per the constitution
- 🧠 Reflection: The constitution clearly establishes the foundational principles for the textbook project.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementing the textbook per the established constitution