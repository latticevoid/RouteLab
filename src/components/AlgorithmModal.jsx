import { useMemo, useState } from 'react';
import { Search, X, Check } from 'lucide-react';
import { ALGORITHMS, CATEGORIES } from '../algorithms/registry.js';

export default function AlgorithmModal({ isOpen, onClose, selectedId, onSelect }) {
  const [query, setQuery] = useState('');
  const [category, setCategory] = useState('All');

  const filtered = useMemo(() => {
    const q = query.trim().toLowerCase();
    return ALGORITHMS.filter((algo) => {
      if (category !== 'All' && algo.category !== category) return false;
      if (!q) return true;
      return (
        algo.name.toLowerCase().includes(q) ||
        algo.category.toLowerCase().includes(q) ||
        algo.desc.toLowerCase().includes(q)
      );
    });
  }, [query, category]);

  if (!isOpen) return null;

  return (
    <div className="modal-backdrop" onClick={onClose}>
      <div className="modal-sheet" onClick={(e) => e.stopPropagation()}>
        <div className="modal-header">
          <div>
            <h2>Select Algorithm</h2>
            <p className="modal-subtitle">{ALGORITHMS.length} pathfinding & routing algorithms</p>
          </div>
          <button className="icon-btn" onClick={onClose} aria-label="Close algorithm picker">
            <X size={20} />
          </button>
        </div>

        <div className="modal-search-wrap">
          <Search size={16} className="search-icon" />
          <input
            type="text"
            className="modal-search"
            placeholder="Search A*, RRT*, CBS, OSPF, Dijkstra..."
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            autoFocus
          />
          {query && (
            <button className="clear-search" onClick={() => setQuery('')} aria-label="Clear search">
              <X size={14} />
            </button>
          )}
        </div>

        <div className="category-pills">
          {CATEGORIES.map((cat) => (
            <button
              key={cat}
              className={`cat-pill ${category === cat ? 'active' : ''}`}
              onClick={() => setCategory(cat)}
            >
              {cat}
            </button>
          ))}
        </div>

        <div className="algo-list">
          {filtered.length === 0 ? (
            <div className="algo-empty">No algorithms match "{query}".</div>
          ) : (
            filtered.map((algo) => {
              const isSelected = algo.id === selectedId;
              return (
                <button
                  key={algo.id}
                  className={`algo-item ${isSelected ? 'selected' : ''}`}
                  onClick={() => {
                    onSelect(algo.id);
                    onClose();
                  }}
                >
                  <div className="algo-item-top">
                    <span className="algo-name">
                      {algo.num > 0 ? `${algo.num}. ` : ''}
                      {algo.name}
                    </span>
                    <div className="algo-badges">
                      {algo.optimal && <span className="badge badge-opt">Optimal</span>}
                      {algo.weighted && <span className="badge badge-w">Weighted</span>}
                      {isSelected && <Check size={16} className="check-icon" />}
                    </div>
                  </div>
                  <div className="algo-meta">
                    <span className="algo-cat">{algo.category}</span>
                    <span className="algo-desc">{algo.desc}</span>
                  </div>
                </button>
              );
            })
          )}
        </div>
      </div>
    </div>
  );
}
