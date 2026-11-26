#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
策略註冊與自動探索模組 (Strategy Registry)
自動掃描 strategies/ 目錄下的策略模組，讓網頁控制台與 StrategyRunner 可動態切換與選擇策略，
無須寫死單一策略。

任何策略模組只要在模組頂層定義以下屬性，就會被自動探索加入選項：
  - STRATEGY_ID: str          (唯一識別碼，例如 "arena_strategy")
  - STRATEGY_NAME: str        (顯示於網頁下拉選單的名稱)
  - STRATEGY_DESC: str        (選填，策略說明)
  - build_strategy() -> List[StrategyStep]
"""

import importlib
import os
from dataclasses import dataclass
from typing import Callable, Dict, List


@dataclass
class StrategyDefinition:
    """已註冊策略的描述資訊與建構函式"""

    strategy_id: str
    name: str
    description: str
    builder: Callable[[], list]


_MANUAL_REGISTRY: Dict[str, StrategyDefinition] = {}


def register_strategy(
    strategy_id: str,
    name: str,
    builder: Callable[[], list],
    description: str = "",
):
    """手動註冊一個策略定義"""
    _MANUAL_REGISTRY[strategy_id] = StrategyDefinition(
        strategy_id=strategy_id,
        name=name,
        description=description,
        builder=builder,
    )


def discover_strategies(reload_modules: bool = False) -> Dict[str, StrategyDefinition]:
    """
    掃描 strategies/ 目錄下所有符合規格的策略檔案，並合併手動註冊的策略。
    回傳依優先順序排列的 {strategy_id: StrategyDefinition} 字典。
    """
    found: Dict[str, StrategyDefinition] = {}
    pkg_dir = os.path.dirname(os.path.abspath(__file__))
    skip_files = {"__init__.py", "runner.py", "registry.py"}

    candidates: List[str] = []
    for fname in sorted(os.listdir(pkg_dir)):
        if fname.endswith(".py") and fname not in skip_files and not fname.startswith("_"):
            candidates.append(fname[:-3])

    for mod_name in candidates:
        full_mod = f"strategies.{mod_name}"
        try:
            mod = importlib.import_module(full_mod)
            if reload_modules:
                mod = importlib.reload(mod)
        except Exception:
            continue

        sid = getattr(mod, "STRATEGY_ID", None)
        builder = getattr(mod, "build_strategy", None)
        if not sid or not callable(builder):
            continue

        sname = getattr(mod, "STRATEGY_NAME", sid)
        sdesc = getattr(mod, "STRATEGY_DESC", "")
        found[str(sid)] = StrategyDefinition(
            strategy_id=str(sid),
            name=str(sname),
            description=str(sdesc),
            builder=builder,
        )

    # 合併手動註冊項目
    for sid, defn in _MANUAL_REGISTRY.items():
        found[sid] = defn

    # 確保預設主策略 arena_strategy 排在第一個 (若存在)
    ordered: Dict[str, StrategyDefinition] = {}
    if "arena_strategy" in found:
        ordered["arena_strategy"] = found["arena_strategy"]
    for sid, defn in found.items():
        if sid not in ordered:
            ordered[sid] = defn

    return ordered
