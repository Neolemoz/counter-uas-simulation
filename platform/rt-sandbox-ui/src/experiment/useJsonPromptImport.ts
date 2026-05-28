import { useCallback } from "react";
import {
  formatImportError,
  type ParseResult,
} from "./experimentImportGuards";

export type PromptJsonImportOptions<T> = {
  promptMessage: string;
  invalidLabel: string;
  parse: (text: string) => ParseResult<T>;
  onSuccess: (data: T) => void;
};

export function promptJsonImport<T>(options: PromptJsonImportOptions<T>): void {
  const text = window.prompt(options.promptMessage);
  if (!text) return;
  const parsed = options.parse(text);
  if (!parsed.ok) {
    window.alert(`Invalid ${options.invalidLabel}: ${formatImportError(parsed.error)}`);
    return;
  }
  options.onSuccess(parsed.data);
}

export function useJsonPromptImport(): <T>(options: PromptJsonImportOptions<T>) => void {
  return useCallback((options) => {
    promptJsonImport(options);
  }, []);
}
