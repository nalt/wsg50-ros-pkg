cmd.register(0xB3);

is_generic = {false, false}

for i=0,finger.count()-1 do
  t = finger.type(i);
  printf("Finger %d is a %s finger\n",i, t );
  if t == "generic" then  
    is_generic[i] = true;
    finger.interface(i, "spi", 140625, 8, 0, 0);
  end
end

function get_finger_data(payload)
    if not is_generic[payload[1]] then
       return false, {} 
    end
    
    while true do
       finger_ready = finger.spi(payload[1], {payload[2]}); 
       if finger_ready[1] == 42 then
           finger_write = {}
           for i=3,#payload do
               finger_write[i-2] = payload[i];  
           end
           data = finger.spi(payload[1], finger_write);
           printf("%d\n",data[1]); 
           break;
       else
           sleep(5);  
       end
    end
    
    return true, data;
end

function process()
    id, payload = cmd.read()
    if id == 0xB3 then
        success, response = get_finger_data(payload);
        if not success then
           printf("Could not get data from finger %d", payload[1]);
        end
        if cmd.online() then
            cmd.send(id, true, true, response); 
        end
    end 
end

while true do
  if cmd.online() then
    if not pcall(process) then 
      print("Error occured");
      sleep(100);
    end
  else
    sleep(100)
  end

end
