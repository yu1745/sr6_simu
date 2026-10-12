import type { Metadata } from "next";
import "./globals.css";

export const metadata: Metadata = {
  title: "SR6 SIMU",
  description: "SR6 六自由度并联机构运动学模拟器",
  other: {
    "codex-preview": "development",
  },
  icons: {
    icon: "/favicon.svg",
    shortcut: "/favicon.svg",
  },
};

export default function RootLayout({
  children,
}: Readonly<{
  children: React.ReactNode;
}>) {
  return (
    <html lang="zh-CN">
      <body className="antialiased">{children}</body>
    </html>
  );
}
