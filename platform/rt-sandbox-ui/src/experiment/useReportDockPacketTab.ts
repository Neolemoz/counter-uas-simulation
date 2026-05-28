import { useEffect, useState } from "react";

export function useReportDockPacketTab(packetTabFocusToken?: number) {
  const [activeTab, setActiveTab] = useState<"slots" | "packet">("slots");
  const [copyStatus, setCopyStatus] = useState<string | null>(null);
  const [packetDownloaded, setPacketDownloaded] = useState(false);
  const [packetTabEverFocused, setPacketTabEverFocused] = useState(false);

  useEffect(() => {
    if (packetTabFocusToken != null && packetTabFocusToken > 0) {
      setActiveTab("packet");
      setPacketTabEverFocused(true);
    }
  }, [packetTabFocusToken]);

  const openPacketTab = () => {
    setActiveTab("packet");
    setPacketTabEverFocused(true);
  };

  return {
    activeTab,
    setActiveTab,
    copyStatus,
    setCopyStatus,
    packetDownloaded,
    setPacketDownloaded,
    packetTabEverFocused,
    openPacketTab,
  };
}
