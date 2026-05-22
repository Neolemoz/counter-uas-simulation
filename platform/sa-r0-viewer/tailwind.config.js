/** @type {import('tailwindcss').Config} */
export default {
  content: ["./index.html", "./src/**/*.{js,ts,jsx,tsx}"],
  theme: {
    extend: {
      maxWidth: {
        publication: "1400px",
      },
      fontSize: {
        caption: ["0.6875rem", { lineHeight: "1.35" }],
      },
      borderRadius: {
        panel: "0.5rem",
      },
    },
  },
  plugins: [],
};
