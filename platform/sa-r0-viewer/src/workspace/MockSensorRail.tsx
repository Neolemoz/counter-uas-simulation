import { CollapsiblePanelSection } from "./CollapsiblePanelSection";
import { MockSensorPanes } from "./MockSensorPanes";

type Props = {
  defaultCollapsed?: boolean;
  focusLabel?: string;
};

export function MockSensorRail({ defaultCollapsed = true, focusLabel }: Props) {
  return (
    <CollapsiblePanelSection
      id="mock.sensors"
      title="Illustrative sensors"
      tier="t5"
      defaultCollapsed={defaultCollapsed}
      helper={
        focusLabel
          ? `Illustrative sync — focus ${focusLabel}. Not live sensor truth.`
          : "Illustrative layout synchronized to replay clock — not live sensor truth."
      }
    >
      <MockSensorPanes />
    </CollapsiblePanelSection>
  );
}
