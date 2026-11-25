import { useEffect, useMemo, useRef, useState } from 'react';
import { ChevronDown, Search, Check } from 'lucide-react';
import { ALGORITHMS, CATEGORIES } from '../algorithms/registry.js';

export default function AlgorithmDropdown({ selectedId, onSelect }) {
  const [open, setOpen] = useState(false);
  const [query, setQuery] = useState('');
  const [category, setCategory] = useState('All');
  const wrapRef = useRef(null);
  const inputRef = useRef(null);

  const selectedAlgo = useMemo(
    () => ALGORITHMS.find((a) => a.id === selectedId) || ALGORITHMS[0],
    [selectedId]
  );

  const filtered = useMemo(() => {
    const q = query.trim().toLowerCase();
    return ALGORITHMS.filter((algo) => {
      if (category !== 'All' && algo.category !== category) return false;
      if (!q) return true;
      return (
        algo.name.toLowerCase().includes(q) ||
        algo.category.toLowerCase().includes(q)
      );
    });
  }, [query, category]);

  useEffect(() => {
    if (!open) return;
    inputRef.current?.focus();

    const handleOutside = (e) => {
      if (wrapRef.current && !wrapRef.current.contains(e.target)) {
        setOpen(false);
      }
    };
    const handleEsc = (e) => {
      if (e.key === 'Escape') setOpen(false);
    };
    document.addEventListener('pointerdown', handleOutside);
    document.addEventListener('keydown', handleEsc);
    return () => {
      document.removeEventListener('pointerdown', handleOutside);
      document.removeEventListener('keydown', handleEsc);
    };
  }, [open]);

  return (
    <div ref={wrapRef} className="algo-dd-wrap">
      <button
        type="button"
        className={`algo-trigger ${open ? 'open' : ''}`}
        onClick={() => setOpen((o) => !o)}
        aria-expanded={open}
        aria-haspopup="listbox"
      >
        <span className="algo-trigger-text">{selectedAlgo.name}</span>
        <ChevronDown size={14} />
      </button>

      {open && (
        <div className="algo-dd-menu" role="listbox">
          <div className="algo-dd-search-wrap">
            <Search size={14} className="algo-dd-search-icon" />
            <input
              ref={inputRef}
              type="text"
              className="algo-dd-search"
              placeholder="Search 103 algorithms..."
              value={query}
              onChange={(e) => setQuery(e.target.value)}
            />
          </div>

          <div className="algo-dd-cats">
            {CATEGORIES.map((cat) => (
              <button
                key={cat}
                type="button"
                className={`algo-dd-cat ${category === cat ? 'active' : ''}`}
                onClick={() => setCategory(cat)}
              >
                {cat}
              </button>
            ))}
          </div>

          <div className="algo-dd-list">
            {filtered.length === 0 ? (
              <div className="algo-dd-empty">No matches</div>
            ) : (
              filtered.map((algo) => {
                const isSelected = algo.id === selectedId;
                return (
                  <button
                    key={algo.id}
                    type="button"
                    role="option"
                    aria-selected={isSelected}
                    className={`algo-dd-item ${isSelected ? 'selected' : ''}`}
                    onClick={() => {
                      onSelect(algo.id);
                      setOpen(false);
                    }}
                  >
                    <div className="algo-dd-item-main">
                      <span className="algo-dd-name">{algo.name}</span>
                      <span className="algo-dd-cat-label">{algo.category}</span>
                    </div>
                    {isSelected && <Check size={14} className="algo-dd-check" />}
                  </button>
                );
              })
            )}
          </div>
        </div>
      )}
    </div>
  );
}
