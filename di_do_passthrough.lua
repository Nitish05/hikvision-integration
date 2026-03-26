-- DI0 -> DO0 passthrough (switch triggers solenoid)
while true do
    local di = GetDI(0, 0)
    if di == 1 then
        SetDO(0, 1, 0, 0)
    else
        SetDO(0, 0, 0, 0)
    end
    WaitMs(10)
end
