import { ExtensionContext } from "@coscene/extension";

import { initStartButton } from "./StartButton";
import { initStatusCard } from "./StatusCard";
import { initTopicArea } from "./TopicArea";
import { initVisionImage } from "./VisionImage";

export function activate(extensionContext: ExtensionContext): void {
  extensionContext.registerPanel({
    name: "status-card",
    initPanel: initStatusCard,
  });
  extensionContext.registerPanel({
    name: "vision-image",
    initPanel: initVisionImage,
  });
  extensionContext.registerPanel({
    name: "start-button",
    initPanel: initStartButton,
  });
  extensionContext.registerPanel({
    name: "topic-area",
    initPanel: initTopicArea,
  });
}
