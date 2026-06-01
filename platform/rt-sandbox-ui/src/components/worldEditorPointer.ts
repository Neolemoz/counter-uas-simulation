import { svgPointToCell, type Cell } from "@/world/gridCoords";

export interface SvgViewTransform {
  viewBoxX: number;
  viewBoxY: number;
  viewBoxWidth: number;
  viewBoxHeight: number;
}

export interface ClientRect {
  left: number;
  top: number;
  width: number;
  height: number;
}

/** Visible SVG content area after default xMidYMid meet letterboxing. */
export function meetContentRect(
  clientRect: ClientRect,
  view: Pick<SvgViewTransform, "viewBoxWidth" | "viewBoxHeight">,
): { left: number; top: number; width: number; height: number } {
  if (view.viewBoxWidth <= 0 || view.viewBoxHeight <= 0) {
    return { left: clientRect.left, top: clientRect.top, width: 0, height: 0 };
  }
  const scale = Math.min(
    clientRect.width / view.viewBoxWidth,
    clientRect.height / view.viewBoxHeight,
  );
  const width = view.viewBoxWidth * scale;
  const height = view.viewBoxHeight * scale;
  return {
    left: clientRect.left + (clientRect.width - width) / 2,
    top: clientRect.top + (clientRect.height - height) / 2,
    width,
    height,
  };
}

/** Map a screen pointer to SVG viewBox coordinates (accounts for zoom/pan + letterboxing). */
export function clientToSvgPoint(
  clientX: number,
  clientY: number,
  clientRect: ClientRect,
  view: SvgViewTransform,
): { x: number; y: number } | null {
  const content = meetContentRect(clientRect, view);
  if (content.width <= 0 || content.height <= 0) return null;
  const relX = (clientX - content.left) / content.width;
  const relY = (clientY - content.top) / content.height;
  return {
    x: view.viewBoxX + relX * view.viewBoxWidth,
    y: view.viewBoxY + relY * view.viewBoxHeight,
  };
}

export function clientToGridCell(
  clientX: number,
  clientY: number,
  clientRect: ClientRect,
  view: SvgViewTransform,
  cellSize: number,
): Cell | null {
  const point = clientToSvgPoint(clientX, clientY, clientRect, view);
  if (!point) return null;
  return svgPointToCell(point.x, point.y, cellSize);
}

/** Keep a grabbed viewBox point under the cursor while panning. */
export function panOriginForGrab(
  grabSvgX: number,
  grabSvgY: number,
  clientX: number,
  clientY: number,
  clientRect: ClientRect,
  viewBoxWidth: number,
  viewBoxHeight: number,
): { x: number; y: number } {
  const content = meetContentRect(clientRect, { viewBoxWidth, viewBoxHeight });
  const relX = content.width > 0 ? (clientX - content.left) / content.width : 0;
  const relY = content.height > 0 ? (clientY - content.top) / content.height : 0;
  return {
    x: grabSvgX - relX * viewBoxWidth,
    y: grabSvgY - relY * viewBoxHeight,
  };
}

/** Prefer native SVG CTM when available (handles transforms exactly). */
export function clientToGridCellFromSvg(
  svg: SVGSVGElement,
  clientX: number,
  clientY: number,
  cellSize: number,
): Cell | null {
  const ctm = svg.getScreenCTM();
  if (!ctm) return null;
  const point = svg.createSVGPoint();
  point.x = clientX;
  point.y = clientY;
  const local = point.matrixTransform(ctm.inverse());
  return svgPointToCell(local.x, local.y, cellSize);
}
