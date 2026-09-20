import 'dart:collection';

/// Fixed-capacity FIFO used for track history, profiler windows and the
/// rolling FPS estimate. Overwrites the oldest element instead of growing, so
/// a long drive cannot turn into an out-of-memory crash.
class RingBuffer<T> extends IterableBase<T> {
  RingBuffer(this.capacity)
      : assert(capacity > 0),
        _store = List<Object?>.filled(capacity, null, growable: false);

  final int capacity;
  final List<Object?> _store;
  int _start = 0;
  int _length = 0;

  @override
  int get length => _length;

  @override
  bool get isEmpty => _length == 0;

  bool get isFull => _length == capacity;

  T operator [](int index) {
    if (index < 0 || index >= _length) {
      throw RangeError.index(index, this, 'index', null, _length);
    }
    return _store[(_start + index) % capacity] as T;
  }

  @override
  T get first {
    if (_length == 0) throw StateError('RingBuffer is empty');
    return this[0];
  }

  @override
  T get last {
    if (_length == 0) throw StateError('RingBuffer is empty');
    return this[_length - 1];
  }

  /// Append, dropping the oldest element when full. Returns the evicted
  /// element, if any, so callers can release resources it owns.
  T? add(T value) {
    T? evicted;
    if (_length == capacity) {
      evicted = _store[_start] as T;
      _store[_start] = value;
      _start = (_start + 1) % capacity;
    } else {
      _store[(_start + _length) % capacity] = value;
      _length++;
    }
    return evicted;
  }

  T? removeFirst() {
    if (_length == 0) return null;
    final T v = _store[_start] as T;
    _store[_start] = null;
    _start = (_start + 1) % capacity;
    _length--;
    return v;
  }

  void clear() {
    for (int i = 0; i < capacity; i++) {
      _store[i] = null;
    }
    _start = 0;
    _length = 0;
  }

  @override
  Iterator<T> get iterator => _RingIterator<T>(this);

  @override
  List<T> toList({bool growable = true}) =>
      List<T>.generate(_length, (int i) => this[i], growable: growable);
}

class _RingIterator<T> implements Iterator<T> {
  _RingIterator(this._buffer);
  final RingBuffer<T> _buffer;
  int _index = -1;

  @override
  T get current => _buffer[_index];

  @override
  bool moveNext() => ++_index < _buffer.length;
}
