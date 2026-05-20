type Props = {
  tags: string[];
  activeTags: string[];
  onToggle: (tag: string) => void;
};

export function ScenarioFilterBar({ tags, activeTags, onToggle }: Props) {
  if (tags.length === 0) return null;
  return (
    <div className="flex flex-wrap gap-1">
      {tags.map((tag) => {
        const active = activeTags.includes(tag);
        return (
          <button
            key={tag}
            type="button"
            className={`rounded px-2 py-0.5 text-xs ${
              active ? "bg-amber-800/80 text-amber-100" : "bg-slate-700 text-slate-400 hover:bg-slate-600"
            }`}
            onClick={() => onToggle(tag)}
          >
            {tag}
          </button>
        );
      })}
    </div>
  );
}
