#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mechatronics-4 自動任務策略模組
"""

from .arena_strategy import ARENA_STEPS, build_arena_strategy
from .registry import StrategyDefinition, discover_strategies, register_strategy
from .runner import StrategyRunner

__all__ = [
    "ARENA_STEPS",
    "build_arena_strategy",
    "StrategyDefinition",
    "discover_strategies",
    "register_strategy",
    "StrategyRunner",
]
