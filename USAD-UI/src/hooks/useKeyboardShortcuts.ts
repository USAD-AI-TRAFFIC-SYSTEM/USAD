import { useEffect } from "react";
import { controlAction } from "../utils/api";

export function useKeyboardShortcuts() {
  useEffect(() => {
    const handler = (e: KeyboardEvent) => {
      // Don't fire when typing in inputs
      if (
        e.target instanceof HTMLInputElement ||
        e.target instanceof HTMLTextAreaElement
      )
        return;

      const key = e.key.toLowerCase();

      switch (key) {
        case "q":
          controlAction("shutdown");
          break;
        case "r":
          controlAction("reset");
          break;
        case "b":
          controlAction("reset-bg");
          break;
        case "a":
          controlAction("auto");
          break;
        case "c":
          controlAction("cycle-camera");
          break;
        case "1":
          controlAction("lane/LANE1");
          break;
        case "2":
          controlAction("lane/LANE2");
          break;
        case "3":
          controlAction("lane/LANE3");
          break;
        case "4":
          controlAction("lane/LANE4");
          break;
        case "f":
          if (!document.fullscreenElement) {
            document.documentElement.requestFullscreen();
          } else {
            document.exitFullscreen();
          }
          break;
        default:
          return;
      }
    };

    window.addEventListener("keydown", handler);
    return () => window.removeEventListener("keydown", handler);
  }, []);
}
