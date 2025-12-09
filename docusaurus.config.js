import {themes as prismThemes} from 'prism-react-renderer';
import path from 'path';

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'A comprehensive guide to robotics, AI, and humanoid control',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://SufyanKhanDilazak.github.io', // Replace with your GitHub username
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub Pages, this should typically be '/your-repo-name/'
  baseUrl: '/Physical-ai-and-humanoid-book', // Replace with your repository name

  // GitHub pages deployment config.
  organizationName: 'SufyanKhanDilazak', // Replace with your GitHub username
  projectName: 'Physical-ai-and-humanoid-book', // Replace with your repository name
  trailingSlash: false,
  onBrokenLinks: 'throw',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
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
            'https://github.com/SufyanKhanDilazak/Physical-ai-and-humanoid-book/tree/main/',
        },
        blog: false, // Disable blog since you're focusing on documentation
        theme: {
          customCss: './src/css/custom.css',
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
          src: 'img/logo.svg', // Add your logo to static/img/
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/SufyanKhanDilazak/Physical-ai-and-humanoid-book',
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
                label: 'Book',
                to: '/docs/module-1-ros2-nervous-system/intro',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/SufyanKhanDilazak/Physical-ai-and-humanoid-book',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: prismThemes.github,
        darkTheme: prismThemes.dracula,
        additionalLanguages: ['python', 'bash', 'xml', 'urdf'],
      },
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: true,
      },
    }),

  plugins: [
    // Add any custom plugins here
    // For example, if you want to add the RAG chatbot plugin later:
    // path.resolve(__dirname, 'src/plugins/docusaurus-ai-plugin'),
  ],
};

export default config;