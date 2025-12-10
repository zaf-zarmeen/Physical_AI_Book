# Quickstart Guide: Physical AI & Humanoid Robotics Textbook

## Getting Started

This guide will help you set up the development environment and build the textbook project.

### Prerequisites

- Node.js 18+ with npm/yarn
- Python 3.11+ with pip
- Git
- ROS 2 Humble Hawksbill (for ROS 2 examples)
- Docker (for containerized environments, optional but recommended)

### Local Development Setup

1. **Clone the repository:**
   ```bash
   git clone <repository-url>
   cd <repository-name>
   ```

2. **Install dependencies:**
   ```bash
   cd book
   npm install
   ```

3. **Run the development server:**
   ```bash
   npm run start
   ```
   This will start the Docusaurus development server at `http://localhost:3000`.

### Building the Textbook

1. **Build for production:**
   ```bash
   npm run build
   ```
   This will create a static site in the `build` directory.

2. **Serve the production build locally:**
   ```bash
   npm run serve
   ```

### Adding New Content

1. **Add a new chapter:**
   - Create a new Markdown file in the appropriate module directory under `book/docs/`
   - Follow the existing structure with frontmatter metadata
   - Update `book/sidebars.js` to include your new chapter in the navigation

2. **Add code examples:**
   - Place Python code examples in the corresponding directory under `book/examples/`
   - Reference them in your Markdown using Docusaurus' code block features

3. **Add diagrams:**
   - Place diagram files in `book/static/img/`
   - Reference them in your Markdown using standard image syntax

### Running Tests

1. **Unit tests for components:**
   ```bash
   npm run test
   ```

2. **Validate code examples:**
   ```bash
   npm run validate-examples
   ```
   This will attempt to execute code examples to ensure they are valid.

### Multi-Platform Simulation Setup

#### ROS 2 Environment
1. Install ROS 2 Humble Hawksbill from the official documentation
2. Source your ROS 2 installation:
   ```bash
   source /opt/ros/humble/setup.bash
   ```
3. Build the example workspaces:
   ```bash
   cd book/examples/ros2
   colcon build
   ```

#### Gazebo Setup
1. Install Gazebo Garden following the official documentation
2. Verify installation:
   ```bash
   gazebo --version
   ```

#### Unity Setup
1. Install Unity Hub and Unity 2022.3 LTS
2. Import the simulation assets from the textbook repository

#### NVIDIA Isaac Sim
1. Install NVIDIA Isaac Sim following the official documentation
2. Set up the required environment variables as specified in the Isaac documentation

### Interactive Features

#### RAG Chatbot Setup
1. Set your OpenAI API key in a `.env` file:
   ```env
   OPENAI_API_KEY=your_api_key_here
   ```
2. The chatbot will automatically index the textbook content when deployed

#### Authentication Setup
1. Configure Better-Auth by adding your environment variables:
   ```env
   NEXTAUTH_URL=http://localhost:3000
   NEXTAUTH_SECRET=your_secret_key
   GITHUB_ID=your_github_client_id
   GITHUB_SECRET=your_github_client_secret
   ```

### Deployment

#### GitHub Pages
The textbook is configured for deployment to GitHub Pages:

1. Update the `baseUrl` and `organizationName`/`projectName` in `docusaurus.config.js` as needed
2. Run the deployment script:
   ```bash
   GIT_USER=<Your GitHub username> USE_SSH=true npm run deploy
   ```

#### Local Deployment
To build and serve locally:
```bash
npm run build
npm run serve
```

### Troubleshooting

1. **Docusaurus fails to start:**
   - Clear the cache: `npx docusaurus clear`
   - Reinstall dependencies: `rm -rf node_modules && npm install`

2. **Code examples not working:**
   - Ensure you have the correct version of Python
   - Check that all ROS 2 dependencies are installed

3. **Translation not working:**
   - Verify that the locale files are correctly placed in `book/i18n/`
   - Check that the language code is supported in `docusaurus.config.js`