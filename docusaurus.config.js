// @ts-check
// `@type` JSDoc annotations allow IDEs and type checkers to infer types
// @ts-ignore
const { themes } = require('prism-react-renderer');

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive course on embodied intelligence and humanoid robotics',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  // Note: For Vercel deployment, use root URL. For GitHub Pages, use github.io URL with repo path
  url: process.env.VERCEL_URL ? `https://${process.env.VERCEL_URL}` : 'https://ami4u87.github.io',
  // Set the /<base>/ pathname under which your site is served
  // For Vercel: use '/', For GitHub Pages: use '/<projectName>/'
  baseUrl: process.env.VERCEL ? '/' : '/Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest/',

  // GitHub pages deployment config.
  organizationName: 'ami4u87', // Usually your GitHub org/user name.
  projectName: 'Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest', // Usually your repo name.
  deploymentBranch: 'gh-pages', // Branch that GitHub pages will deploy from.

  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/ami4u87/Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest/tree/master/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'docsSidebar',
            position: 'left',
            label: 'Docs',
          },
          {
            href: 'https://github.com/ami4u87/Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Docs',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/physical-ai-humanoid-robotics',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/physical-ai-humanoid-robotics',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/ami4u87/Hackathorn-_book_Physical-AI-_or_Humanoid-Robotics-Course-latest',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Course. Built with Docusaurus.`,
      },
      prism: {
        theme: themes.github,
        darkTheme: themes.dracula,
        additionalLanguages: ['python', 'bash', 'docker', 'yaml'],
      },
    }),
};

module.exports = config;