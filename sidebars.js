module.exports = {
  docsSidebar: [
    {
      type: 'doc',
      id: 'intro',
    },
    {
      type: 'category',
      label: 'Module 1: ROS 2 Fundamentals',
      items: [
        'module-1-ros2/index',
        'module-1-ros2/installation',
        'module-1-ros2/pubsub',
        'module-1-ros2/services',
        'module-1-ros2/actions',
        'module-1-ros2/parameters',
        'module-1-ros2/launch',
        'module-1-ros2/tf2',
        'module-1-ros2/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments',
      items: [
        'module-2-simulation/index',
        'module-2-simulation/gazebo',
        'module-2-simulation/unity',
        'module-2-simulation/isaac',
        'module-2-simulation/physics',
        'module-2-simulation/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Perception & Control',
      items: [
        'module-3-perception/index',
        'module-3-perception/vision-depth',
        'module-3-perception/object-detection',
        'module-3-perception/pose-tf',
        'module-3-perception/rviz',
        'module-3-perception/control-integration',
        'module-3-perception/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action',
      items: [
        'module-4-vla/index',
        'module-4-vla/gpt4-vision',
        'module-4-vla/multimodal',
        'module-4-vla/action-sequences',
        'module-4-vla/validation',
        'module-4-vla/ambiguity',
        'module-4-vla/exercises'
      ],
    },
    {
      type: 'category',
      label: 'Module 5: Capstone Project',
      items: [
        'module-5-capstone/index',
        'module-5-capstone/integration',
        'module-5-capstone/voice-interface',
        'module-5-capstone/final-demo'
      ],
    },
  ],
};