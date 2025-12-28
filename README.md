# Physical AI & Humanoid Robotics Course

A comprehensive, hands-on course covering embodied intelligence, humanoid robotics, and autonomous systems. This interactive documentation site includes 5 complete modules with exercises, code examples, and an integrated AI chatbot trained on course content.

## Overview

This course provides a progressive learning path from ROS 2 fundamentals to building autonomous humanoid robots with vision-language-action (VLA) capabilities. Each module builds upon the previous one, culminating in a voice-commanded manipulation system.

**Live Site:** [https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app](https://hackathorn-book-physical-ai-or-huma-ebon.vercel.app)

## Features

- **Interactive Documentation**: Built with Docusaurus 3.1 for a modern learning experience
- **AI Chatbot**: RAG-powered assistant trained on 659+ course content chunks
- **Hands-on Exercises**: Practical coding challenges for each module
- **Multi-Platform Support**: Covers Gazebo, Unity, and NVIDIA Isaac Sim
- **Production-Ready Code**: Real-world examples using ROS 2 Humble

## Course Modules

### Module 1: ROS 2 Fundamentals (Weeks 1-5)
- Installation and workspace setup
- Publishers and Subscribers
- Services and Clients
- Actions and Action Servers
- Parameters and Launch Files
- TF2 Coordinate Transforms

### Module 2: Simulation Environments (Weeks 6-7)
- Gazebo Harmonic integration
- Unity with ROS-TCP-Connector
- NVIDIA Isaac Sim
- Physics validation and testing

### Module 3: Perception & Control (Weeks 8-10)
- Vision and depth perception
- Object detection (YOLO, classical CV)
- Pose estimation and TF tree
- RViz2 visualization
- Control system integration

### Module 4: Vision-Language-Action (Weeks 11-12)
- GPT-4 Vision integration
- Multi-modal input processing
- Structured action generation
- Action feasibility validation
- Ambiguity handling

### Module 5: Capstone Project (Week 13)
- Full system integration
- Voice command interface
- Autonomous manipulation demo

## Technology Stack

- **Framework**: Docusaurus 3.1
- **Robotics**: ROS 2 Humble
- **Simulation**: Gazebo Harmonic, Unity, NVIDIA Isaac Sim
- **AI/ML**: GPT-4 Vision, Gemini 1.5 Flash
- **Vector DB**: Qdrant
- **Deployment**: Vercel (production), GitHub Pages (preview)
- **Languages**: Python, JavaScript/React

## Getting Started

### Prerequisites

- Node.js >= 18.0
- npm or yarn
- (Optional) ROS 2 Humble for hands-on exercises

### Installation

```bash
# Clone the repository
git clone https://github.com/ami4u87/Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest.git
cd Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest

# Install dependencies
npm install

# Start the development server
npm start
```

The site will open at `http://localhost:3000`

### Build for Production

```bash
npm run build
npm run serve
```

## AI Chatbot

The integrated AI chatbot provides instant answers about course content:

- **Technology**: RAG (Retrieval-Augmented Generation) with Qdrant vector database
- **Model**: Google Gemini 1.5 Flash
- **Coverage**: All course modules, exercises, and code examples
- **Access**: Available at `/chatbot` or via floating button on all pages

### Running Chatbot Locally

See [physical-ai-chatbot/README.md](./physical-ai-chatbot/README.md) for detailed setup instructions.

## Deployment

### Vercel (Production)
```bash
npm run deploy
```

### GitHub Pages (Preview)
```bash
GITHUB_PAGES=true npm run build
npm run deploy
```

## Project Structure

```
.
├── docs/                    # Course documentation
│   ├── module-1-ros2/      # ROS 2 fundamentals
│   ├── module-2-simulation/ # Simulation environments
│   ├── module-3-perception/ # Perception & control
│   ├── module-4-vla/       # Vision-Language-Action
│   └── module-5-capstone/  # Capstone project
├── physical-ai-chatbot/    # AI chatbot backend
├── src/                    # Custom React components
├── static/                 # Static assets
├── docusaurus.config.js    # Site configuration
└── sidebars.js            # Navigation structure
```

## Contributing

Contributions are welcome! Please feel free to submit issues or pull requests.

1. Fork the repository
2. Create your feature branch (`git checkout -b feature/amazing-feature`)
3. Commit your changes (`git commit -m 'Add some amazing feature'`)
4. Push to the branch (`git push origin feature/amazing-feature`)
5. Open a Pull Request

## License

This project is available for educational purposes.

## Acknowledgments

- Built with [Docusaurus](https://docusaurus.io/)
- ROS 2 resources from [ros.org](https://www.ros.org/)
- AI integrations powered by Google Gemini and OpenAI GPT-4

## Support

For questions or support:
- Open an issue on GitHub
- Use the integrated AI chatbot for course-related questions
- Check the documentation at each module's index page

---

**Built with ❤️ for robotics education**
