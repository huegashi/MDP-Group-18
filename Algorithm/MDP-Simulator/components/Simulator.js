"use client"
import { useState, useEffect, useRef } from "react"
import QueryAPI from "./QueryAPI"

const Direction = { NORTH: 0, EAST: 2, SOUTH: 4, WEST: 6, SKIP: 8 }
const ObDirection = { NORTH: 0, EAST: 2, SOUTH: 4, WEST: 6, SKIP: 8 }

const DirectionToString = {
  0: "Up",
  2: "Right",
  4: "Down",
  6: "Left",
  8: "None",
}

const transformCoord = (x, y) => ({ x: 19 - y, y: x })
function classNames(...classes) {
  return classes.filter(Boolean).join(" ")
}

export default function Simulator() {
  const [robotState, setRobotState] = useState({ x: 1, y: 1, d: Direction.NORTH, s: -1 })
  const [robotX, setRobotX] = useState(1)
  const [robotY, setRobotY] = useState(1)
  const [robotDir, setRobotDir] = useState(0)
  const [obstacles, setObstacles] = useState([])
  const [obXInput, setObXInput] = useState(0)
  const [obYInput, setObYInput] = useState(0)
  const [directionInput, setDirectionInput] = useState(ObDirection.NORTH)
  const [isComputing, setIsComputing] = useState(false)
  const [path, setPath] = useState([])
  const [commands, setCommands] = useState([])
  const [page, setPage] = useState(0)
  const [isAnimating, setIsAnimating] = useState(false)
  const [animationSpeed, setAnimationSpeed] = useState(1000) // milliseconds between steps
  const animationRef = useRef(null)

  const generateNewID = () => {
    while (true) {
      const new_id = Math.floor(Math.random() * 10) + 1
      let ok = true
      for (const ob of obstacles) {
        if (ob.id === new_id) {
          ok = false
          break
        }
      }
      if (ok) return new_id
    }
  }

  const generateRobotCells = () => {
    const robotCells = []
    let markerX = 0,
      markerY = 0

    if (Number(robotState.d) === Direction.NORTH) markerY++
    else if (Number(robotState.d) === Direction.EAST) markerX++
    else if (Number(robotState.d) === Direction.SOUTH) markerY--
    else if (Number(robotState.d) === Direction.WEST) markerX--

    for (let i = -1; i < 2; i++) {
      for (let j = -1; j < 2; j++) {
        const coord = transformCoord(robotState.x + i, robotState.y + j)
        if (markerX === i && markerY === j)
          robotCells.push({ x: coord.x, y: coord.y, d: robotState.d, s: robotState.s })
        else robotCells.push({ x: coord.x, y: coord.y, d: null, s: -1 })
      }
    }
    return robotCells
  }

  // Inputs
  const onChangeX = (e) => {
    if (Number.isInteger(Number(e.target.value))) {
      const nb = Number(e.target.value)
      if (0 <= nb && nb < 20) {
        setObXInput(nb)
        return
      }
    }
    setObXInput(0)
  }
  const onChangeY = (e) => {
    if (Number.isInteger(Number(e.target.value))) {
      const nb = Number(e.target.value)
      if (0 <= nb && nb <= 19) {
        setObYInput(nb)
        return
      }
    }
    setObYInput(0)
  }
  const onChangeRobotX = (e) => {
    if (Number.isInteger(Number(e.target.value))) {
      const nb = Number(e.target.value)
      if (1 <= nb && nb < 19) {
        setRobotX(nb)
        return
      }
    }
    setRobotX(1)
  }
  const onChangeRobotY = (e) => {
    if (Number.isInteger(Number(e.target.value))) {
      const nb = Number(e.target.value)
      if (1 <= nb && nb < 19) {
        setRobotY(nb)
        return
      }
    }
    setRobotY(1)
  }
  const onDirectionInputChange = (e) => setDirectionInput(Number(e.target.value))
  const onRobotDirectionInputChange = (e) => setRobotDir(e.target.value)

  // Actions
  const onClickObstacle = () => {
    if (!obXInput && !obYInput) return
    setObstacles((prev) => [...prev, { x: obXInput, y: obYInput, d: directionInput, id: generateNewID() }])
  }

  const onClickRobot = () => {
    setRobotState({ x: robotX, y: robotY, d: robotDir, s: -1 })
  }

  const onRemoveObstacle = (ob) => {
    if (path.length > 0 || isComputing) return
    setObstacles((prev) => prev.filter((o) => o.id !== ob.id)) // FIX: remove by id
  }

  const compute = () => {
    setIsComputing(true)
    QueryAPI.query(obstacles, robotX, robotY, robotDir, (data, err) => {
      if (data) {
        setPath(data.data.path)
        const cmds = []
        for (const x of data.data.commands) if (!x.startsWith("SNAP")) cmds.push(x)
        setCommands(cmds)
      }
      setIsComputing(false)
    })
  }

  const onResetAll = () => {
    stopAnimation()
    setRobotX(1)
    setRobotDir(0)
    setRobotY(1)
    setRobotState({ x: 1, y: 1, d: Direction.NORTH, s: -1 })
    setPath([])
    setCommands([])
    setPage(0)
    setObstacles([])
  }

  const onReset = () => {
    stopAnimation()
    setRobotX(1)
    setRobotDir(0)
    setRobotY(1)
    setRobotState({ x: 1, y: 1, d: Direction.NORTH, s: -1 })
    setPath([])
    setCommands([])
    setPage(0)
  }

  // Animation control functions
  const startAnimation = () => {
    if (path.length === 0 || page >= path.length - 1) return
    setIsAnimating(true)
  }

  const stopAnimation = () => {
    setIsAnimating(false)
    if (animationRef.current) {
      clearTimeout(animationRef.current)
      animationRef.current = null
    }
  }

  const resetToStart = () => {
    stopAnimation()
    setPage(0)
  }

  // useEffect for animation logic
  useEffect(() => {
    if (isAnimating && page < path.length - 1) {
      animationRef.current = setTimeout(() => {
        setPage((prev) => prev + 1)
      }, animationSpeed)
    } else if (page >= path.length - 1) {
      setIsAnimating(false)
    }

    return () => {
      if (animationRef.current) {
        clearTimeout(animationRef.current)
        animationRef.current = null
      }
    }
  }, [isAnimating, page, path.length, animationSpeed])

  // Grid
  const renderGrid = () => {
    const rows = []
    const robotCells = generateRobotCells()

    for (let i = 0; i < 20; i++) {
      const cells = [
        // y-axis label (no box)
        <td key={i} className="axis w-5 h-5 md:w-8 md:h-8">
          <span className="font-bold text-[0.6rem] md:text-base">{19 - i}</span>
        </td>,
      ]

      for (let j = 0; j < 20; j++) {
        let foundOb = null,
          foundRobotCell = null

        for (const ob of obstacles) {
          const t = transformCoord(ob.x, ob.y)
          if (t.x === i && t.y === j) {
            foundOb = ob
            break
          }
        }

        if (!foundOb) {
          for (const cell of robotCells) {
            if (cell.x === i && cell.y === j) {
              foundRobotCell = cell
              break
            }
          }
        }

        const originalX = j
        const originalY = 19 - i
        const isStartZone = originalX >= 0 && originalX <= 3 && originalY >= 0 && originalY <= 3

        if (foundOb) {
          let borderClass = ""
          if (!isStartZone) {
            if (foundOb.d === ObDirection.NORTH) borderClass = "border-t-4 border-t-red-500"
            else if (foundOb.d === ObDirection.SOUTH) borderClass = "border-b-4 border-b-red-500"
            else if (foundOb.d === ObDirection.EAST) borderClass = "border-r-4 border-r-red-500"
            else if (foundOb.d === ObDirection.WEST) borderClass = "border-l-4 border-l-red-500"
          }

          cells.push(<td key={j} className={`border w-5 h-5 md:w-8 md:h-8 bg-blue-700 ${borderClass}`} />)
        } else if (foundRobotCell) {
          if (foundRobotCell.d !== null) {
            cells.push(
              <td
                key={j}
                className={`border w-5 h-5 md:w-8 md:h-8 ${foundRobotCell.s !== -1 ? "bg-red-500" : "bg-yellow-300"}`}
              />,
            )
          } else {
            cells.push(<td key={j} className="bg-green-600 border w-5 h-5 md:w-8 md:h-8" />)
          }
        } else {
          const bgClass = isStartZone ? "bg-sky-200" : ""
          cells.push(<td key={j} className={`border w-5 h-5 md:w-8 md:h-8 ${bgClass}`} />)
        }
      }
      rows.push(<tr key={19 - i}>{cells}</tr>)
    }

    // x-axis labels row (no boxes)
    const xAxis = [<td key={0} className="axis" />]
    for (let i = 0; i < 20; i++) {
      xAxis.push(
        <td key={`x-${i}`} className="axis w-5 h-5 md:w-8 md:h-8">
          <span className="font-bold text-[0.6rem] md:text-base">{i}</span>
        </td>,
      )
    }
    rows.push(<tr key={20}>{xAxis}</tr>)
    return rows
  }

  useEffect(() => {
    if (page >= path.length) return
    setRobotState(path[page])
  }, [page, path])

  return (
    <div className="min-h-screen bg-gradient-to-br from-slate-900 via-slate-800 to-slate-900 p-4">
      <div className="max-w-7xl mx-auto">
        <div className="text-center mb-12">
          <h1 className="text-5xl md:text-6xl font-bold text-white mb-4">MDP Algorithm Simulator</h1>
        </div>

        <div className="grid grid-cols-1 xl:grid-cols-3 gap-8">
          <div className="xl:col-span-1 space-y-6">
            {/* Robot Position card */}
            <div className="bg-slate-800/50 backdrop-blur-sm border border-slate-700 rounded-2xl p-6 shadow-2xl">
              <div className="flex items-center gap-3 mb-6">
                <div className="w-3 h-3 bg-green-400 rounded-full animate-pulse"></div>
                <h2 className="text-xl font-semibold text-white">Robot Position</h2>
              </div>
              <div className="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">X Coordinate</label>
                  <input
                    onChange={onChangeRobotX}
                    type="number"
                    min="1"
                    max="18"
                    className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                  />
                </div>
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">Y Coordinate</label>
                  <input
                    onChange={onChangeRobotY}
                    type="number"
                    min="1"
                    max="18"
                    className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                  />
                </div>
              </div>
              <div className="mb-4">
                <label className="block text-sm font-medium text-slate-300 mb-2">Direction</label>
                <select
                  onChange={onRobotDirectionInputChange}
                  value={robotDir}
                  className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                >
                  <option value={ObDirection.NORTH}>Up</option>
                  <option value={ObDirection.SOUTH}>Down</option>
                  <option value={ObDirection.WEST}>Left</option>
                  <option value={ObDirection.EAST}>Right</option>
                </select>
              </div>
              <button
                className="w-full bg-gradient-to-r from-green-500 to-emerald-600 hover:from-green-600 hover:to-emerald-700 text-white font-semibold py-3 px-4 rounded-lg transition-all duration-200 shadow-lg hover:shadow-xl"
                onClick={onClickRobot}
              >
                Set Robot Position
              </button>
            </div>

            {/* Add Obstacles card */}
            <div className="bg-slate-800/50 backdrop-blur-sm border border-slate-700 rounded-2xl p-6 shadow-2xl">
              <div className="flex items-center gap-3 mb-6">
                <div className="w-3 h-3 bg-blue-400 rounded-full"></div>
                <h2 className="text-xl font-semibold text-white">Add Obstacles</h2>
              </div>
              <div className="grid grid-cols-2 gap-4 mb-4">
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">X Coordinate</label>
                  <input
                    onChange={onChangeX}
                    type="number"
                    min="0"
                    max="19"
                    className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                  />
                </div>
                <div>
                  <label className="block text-sm font-medium text-slate-300 mb-2">Y Coordinate</label>
                  <input
                    onChange={onChangeY}
                    type="number"
                    min="0"
                    max="19"
                    className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                  />
                </div>
              </div>
              <div className="mb-4">
                <label className="block text-sm font-medium text-slate-300 mb-2">Direction</label>
                <select
                  onChange={onDirectionInputChange}
                  value={directionInput}
                  className="w-full px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                >
                  <option value={ObDirection.NORTH}>Up</option>
                  <option value={ObDirection.SOUTH}>Down</option>
                  <option value={ObDirection.WEST}>Left</option>
                  <option value={ObDirection.EAST}>Right</option>
                  <option value={ObDirection.SKIP}>None</option>
                </select>
              </div>
              <button
                className="w-full bg-gradient-to-r from-blue-500 to-cyan-600 hover:from-blue-600 hover:to-cyan-700 text-white font-semibold py-3 px-4 rounded-lg transition-all duration-200 shadow-lg hover:shadow-xl"
                onClick={onClickObstacle}
              >
                Add Obstacle
              </button>
            </div>

            {obstacles.length > 0 && (
              <div className="bg-slate-800/50 backdrop-blur-sm border border-slate-700 rounded-2xl p-6 shadow-2xl">
                <div className="flex items-center gap-3 mb-4">
                  <div className="w-3 h-3 bg-red-400 rounded-full"></div>
                  <h3 className="text-lg font-semibold text-white">Current Obstacles</h3>
                </div>
                <div className="grid grid-cols-1 gap-3">
                  {obstacles.map((ob) => (
                    <div
                      key={ob.id}
                      className="flex items-center justify-between bg-slate-700/50 border border-slate-600 rounded-lg p-3 hover:bg-slate-700/70 transition-colors"
                    >
                      <div className="text-slate-300 text-sm">
                        <span className="font-mono">
                          ({ob.x}, {ob.y})
                        </span>{" "}
                        - {DirectionToString[ob.d]}
                      </div>
                      <button
                        className="text-red-400 hover:text-red-300 hover:bg-red-500/20 rounded-full p-1 transition-colors"
                        onClick={() => onRemoveObstacle(ob)}
                      >
                        <svg className="w-4 h-4" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M6 18L18 6M6 6l12 12" />
                        </svg>
                      </button>
                    </div>
                  ))}
                </div>
              </div>
            )}

            <div className="grid grid-cols-1 gap-3">
              <button
                className="bg-gradient-to-r from-red-500 to-pink-600 hover:from-red-600 hover:to-pink-700 text-white font-semibold py-3 px-4 rounded-lg transition-all duration-200 shadow-lg hover:shadow-xl"
                onClick={onResetAll}
              >
                Reset All
              </button>
              <button
                className="bg-gradient-to-r from-yellow-500 to-orange-600 hover:from-yellow-600 hover:to-orange-700 text-white font-semibold py-3 px-4 rounded-lg transition-all duration-200 shadow-lg hover:shadow-xl"
                onClick={onReset}
              >
                Reset Robot
              </button>
              <button
                className="bg-gradient-to-r from-purple-500 to-indigo-600 hover:from-purple-600 hover:to-indigo-700 text-white font-semibold py-3 px-4 rounded-lg transition-all duration-200 shadow-lg hover:shadow-xl"
                onClick={compute}
              >
                {isComputing ? "Computing..." : "Start Simulation"}
              </button>
            </div>
          </div>

          <div className="xl:col-span-2 space-y-6">
            {/* Path navigation */}
            {path.length > 0 && (
              <div className="bg-slate-800/50 backdrop-blur-sm border border-slate-700 rounded-2xl p-6 shadow-2xl">
                <div className="flex flex-col lg:flex-row items-center justify-between gap-4">
                  <div className="flex items-center gap-4">
                    <button
                      className="bg-slate-700 hover:bg-slate-600 text-white p-3 rounded-full transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                      disabled={page === 0 || isAnimating}
                      onClick={() => setPage(page - 1)}
                    >
                      <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M15 19l-7-7 7-7" />
                      </svg>
                    </button>
                    <div className="text-center">
                      <div className="text-white font-semibold">
                        Step {page + 1} of {path.length}
                      </div>
                      <div className="text-slate-400 text-sm font-mono bg-slate-700 px-3 py-1 rounded-md mt-1">
                        {commands[page]}
                      </div>
                    </div>
                    <button
                      className="bg-slate-700 hover:bg-slate-600 text-white p-3 rounded-full transition-colors disabled:opacity-50 disabled:cursor-not-allowed"
                      disabled={page === path.length - 1 || isAnimating}
                      onClick={() => setPage(page + 1)}
                    >
                      <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M9 5v14l11-7z" />
                      </svg>
                    </button>
                  </div>

                  <div className="flex items-center gap-3">
                    <button
                      className="bg-green-600 hover:bg-green-700 text-white p-3 rounded-full transition-colors disabled:opacity-50"
                      disabled={page >= path.length - 1}
                      onClick={isAnimating ? stopAnimation : startAnimation}
                    >
                      {isAnimating ? (
                        <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                          <path strokeLinecap="round" strokeLinejoin="round" strokeWidth={2} d="M10 9v6m4-6v6" />
                        </svg>
                      ) : (
                        <svg className="w-5 h-5" fill="currentColor" viewBox="0 0 24 24">
                          <path d="M8 5v14l11-7z" />
                        </svg>
                      )}
                    </button>
                    <button
                      className="bg-blue-600 hover:bg-blue-700 text-white p-3 rounded-full transition-colors"
                      onClick={resetToStart}
                    >
                      <svg className="w-5 h-5" fill="none" stroke="currentColor" viewBox="0 0 24 24">
                        <path
                          strokeLinecap="round"
                          strokeLinejoin="round"
                          strokeWidth={2}
                          d="M12.066 11.2a1 1 0 000 1.6l5.334 4A1 1 0 0019 16V8a1 1 0 00-1.6-.8l-5.334 4zM4.066 11.2a1 1 0 000 1.6l5.334 4A1 1 0 0011 16V8a1 1 0 00-1.6-.8l-5.334 4z"
                        />
                      </svg>
                    </button>
                    <div className="flex items-center gap-2">
                      <span className="text-slate-300 text-sm">Speed:</span>
                      <select
                        value={animationSpeed}
                        onChange={(e) => setAnimationSpeed(Number(e.target.value))}
                        className="px-3 py-2 bg-slate-700 border border-slate-600 rounded-lg text-white text-sm focus:ring-2 focus:ring-blue-500 focus:border-transparent"
                        disabled={isAnimating}
                      >
                        <option value={2000}>Slow</option>
                        <option value={1000}>Normal</option>
                        <option value={500}>Fast</option>
                        <option value={250}>Very Fast</option>
                      </select>
                    </div>
                  </div>
                </div>
              </div>
            )}

            <div className="bg-slate-800/50 backdrop-blur-sm border border-slate-700 rounded-2xl p-6 shadow-2xl">
              <div className="flex items-center gap-3 mb-6">
                <div className="w-3 h-3 bg-cyan-400 rounded-full"></div>
                <h3 className="text-xl font-semibold text-white">Simulation Grid</h3>
              </div>
              <div className="overflow-auto">
                <table className="border-separate border-spacing-0 mx-auto">
                  <tbody>{renderGrid()}</tbody>
                </table>
              </div>
            </div>
          </div>
        </div>
      </div>
    </div>
  )
}
